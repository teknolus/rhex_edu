#!/usr/bin/env python3

import sys
import errno
import socket
import threading
import rclpy
import select
import time
from miscellaneous import constrain_angle, remove_spaces
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from control_msgs.action import FollowJointTrajectory
from launch.substitutions import LaunchConfiguration
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu, PointCloud2
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from gazebo_msgs.srv import GetModelState, GetEntityState
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')
        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        # self.subPos = Subscriber(self, Odometry,'/odom/robot_pos')
        self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints = Subscriber(self, JointState, '/joint_states')
        self.subIMU = Subscriber(self, Imu, '/imu/data')

        # Pozisyon verisini diğer veriler ile senkronize etmeye çalışmak kontrolcünün performansını
        # düşürüyor ya da bozuyor. Bu yüzden pozisyon verisini ayrıca kaydediyoruz.
        TimeSynchronizer([
            # self.subPos,
            self.subJoints,
            self.subIMU
            ], 
            10
        ).registerCallback(self.callback)

        # Declare socket parameters and their default values for communication with RosNet
        self.declare_parameter('host', "127.0.0.1")     # Standard loopback interface address (localhost)
        self.declare_parameter('port', 1256)            # Port to listen on (non-privileged ports are > 1023)

        # Retrieve socket parameters since they can be changed while calling the launch file
        self.host = self.get_parameter("host").value    
        self.port = self.get_parameter("port").value

        # Create a STREAM socket and start listening
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen()
        self.get_logger().info("Server listening on {}:{}".format(self.host, self.port))

        self.currTime = 0
        self.currPos = np.zeros(6)
        self.currVel = np.zeros(6)
        self.currTorq = np.zeros(6)
        self.currPose = np.zeros(4)
        self.globalPos = np.zeros(3)
        self.clientMsg = ""
        self.newdata = False

        self.cmd_time = 0
        self.cmd_pos = np.zeros(6)
        self.cmd_kp = np.zeros(6)
        self.cmd_vel = np.zeros(6)
        self.cmd_kd = np.zeros(6)
        self.cmd_tau = np.zeros(6)
        self.cmd_enabled = False

        self.lock = threading.Lock()
        self.exitThread = False  

        self.get_logger().info("\n\n---------------------- The node is ready! ----------------------\n\n")

    def callback_position(self, msg):
        with self.lock:
            self.globalPos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])

    def callback(self,
                #  msgOdom: Odometry,
                 msgJoint: JointState,
                 msgIMU: Imu
                ):
        
        with self.lock:
            self.newdata = True
            self.currTime = msgJoint.header.stamp.sec + msgJoint.header.stamp.nanosec/1e9
            self.currPos = constrain_angle(np.array([*msgJoint.position]))
            self.currVel = np.array([*msgJoint.velocity])
            self.currTorq = np.array([*msgJoint.effort])
            self.currPose = np.array([
                msgIMU.orientation.x,
                msgIMU.orientation.y,
                msgIMU.orientation.z,
                msgIMU.orientation.w
            ])
            # self.globalPos = np.array([
            #     msgOdom.pose.pose.position.x,
            #     msgOdom.pose.pose.position.y,
            #     msgOdom.pose.pose.position.z
            # ])

            # Change the leg order so that it matches with the RosNet configuration
            self.currPos = self.currPos[[4, 2, 0, 5, 3, 1]]     
            self.currVel = self.currVel[[4, 2, 0, 5, 3, 1]]
            self.currTorq = self.currTorq[[4, 2, 0, 5, 3, 1]]

            self.clientMsg = "".join([
                f"t: {self.currTime}, ",
                f"hp: {-self.currPos}, ",
                f"hv: {-self.currVel}, ",
                f"ht: {-self.currTorq}, ",
                f"ro: {self.currPose}, ",
                f"rp: {self.globalPos}"
            ])
            self.clientMsg = remove_spaces(self.clientMsg)
            self.clientMsg = self.clientMsg.replace("\n", "")+"\n"

    def run(self):
        self.reqThread = threading.Thread(target=self.handle_request)
        self.reqThread.start()

        torque = Float64MultiArray()
        torque.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info("--------------------------- RUN ------------------------")
        while rclpy.ok():
            try:
                if self.newdata:
                    torque.data = self.compute_controls()
                    self.publisher.publish(torque)
                rclpy.spin_once(self)

                if self.exitThread:
                    break

            except KeyboardInterrupt:
                self.finish()
                break

    def compute_controls(self):
        with self.lock:
            if (self.cmd_enabled):
                posErr = (-self.cmd_pos) - self.currPos
                posErr = np.mod(posErr+np.pi, 2*np.pi) - np.pi
                velErr = (-self.cmd_vel) - self.currVel
                commTorque = np.multiply(self.cmd_kp, posErr) + np.multiply(self.cmd_kd, velErr) - self.cmd_tau
                np.clip(commTorque, -20, 20, out=commTorque)
            else:
                commTorque = np.zeros(6)
            # self.get_logger().info("t= ", self.currTime, ": ", list(commTorque))
            self.newdata = False
        return list(commTorque[[2, 5, 1, 4, 0, 3]])

    def handle_request(self):
        torque = Float64MultiArray()
        self.socket.settimeout(0.1)
        isClientClosed = False

        while not self.exitThread:
            # Block to accept an incoming connection
            try:
                clientSocket, client_addr = self.socket.accept()
                clientSocket.settimeout(0.1)
            except socket.timeout:
                # self.get_logger().info("Connection timed out, trying again!")
                continue
            except Exception as e:
                self.get_logger().info("Exception during the accept() call. Exiting.")
                isClientClosed = True
                self.exitThread = True
                continue

            isClientClosed = False
            self.get_logger().info("Client connected from {}:{}".format(client_addr[0], client_addr[1]))

            lastmsg = ""
            newcmd = False
            while (not isClientClosed and not self.exitThread):
                try:
                    messageClient = lastmsg + clientSocket.recv(1024).decode()
                    msgs = messageClient.split('\n')
                    for idx, m in enumerate(msgs):
                        if (idx == len(msgs)-1):
                            lastmsg = m
                            break
                        if m.lower().startswith("disconnect"):
                            self.get_logger().info("Received DISCONNECT")
                            clientSocket.shutdown(socket.SHUT_RDWR)
                            clientSocket.close()
                            isClientClosed = True
                            break

                        elif m.lower().startswith("state"):
                            # self.get_logger().info("Received STATE")
                            with self.lock:
                                # self.get_logger().info(self.clientMsg.encode())
                                clientSocket.sendall(self.clientMsg.encode())

                        elif m.lower().startswith("enable"):
                            newcmd = True
                            with self.lock:
                                self.cmd_enabled = True
                                self.get_logger().info("Received ENABLE")

                        elif m.lower().startswith("disable"):
                            with self.lock:
                                self.cmd_enabled = False
                                torque.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                                self.get_logger().info("Received DISABLE")

                        elif m.lower().startswith("cmdtau"):
                            newcmd = True
                            with self.lock:
                                self.cmd_time = self.currTime
                                self.cmd_tau = np.array([float(num) for num in m.split()[1:]])
                                # self.get_logger().info("Received torques: ", list(self.cmd_tau.data))

                        elif m.lower().startswith("cmdpos"):
                            newcmd = True
                            with self.lock:
                                self.cmd_time = self.currTime
                                self.cmd_pos = np.array([float(num) for num in m.split()[1:]])
                                # self.get_logger().info("Received position ", list(self.cmd_pos.data))

                        elif m.lower().startswith("cmdkp"):
                            newcmd = True
                            with self.lock:
                                self.cmd_time = self.currTime
                                self.cmd_kp = np.array([float(num) for num in m.split()[1:]])
                                self.get_logger().info(f"Received KP {list(self.cmd_kp.data)}")

                        elif m.lower().startswith("cmdvel"):
                            newcmd = True
                            with self.lock:
                                self.cmd_time = self.currTime
                                self.cmd_vel = np.array([float(num) for num in m.split()[1:]])
                                # self.get_logger().info("Received velocity ", list(self.cmd_vel.data))

                        elif m.lower().startswith("cmdkd"):
                            newcmd = True
                            with self.lock:
                                self.cmd_time = self.currTime
                                self.cmd_kd = np.array([float(num) for num in m.split()[1:]])
                                self.get_logger().info(f"Received KD {list(self.cmd_kd.data)}")

                        else:
                            self.get_logger().info("[WARNING] Invalid command:"+m)
                            # clientSocket.sendall("\n".encode())

                    if (self.cmd_enabled and newcmd):
                        torque.data = self.compute_controls()
                        self.publisher.publish(torque)

                except socket.timeout:
                    continue
                except UnicodeDecodeError:
                    self.get_logger().info("Invalid unicode input")
                    continue
                except IOError as e:
                    if e.errno == errno.EPIPE:
                        self.get_logger().info(
                            "Pipe error, closing connection and waiting for another client!")
                        clientSocket.shutdown(socket.SHUT_RDWR)
                        clientSocket.close()
                        isClientClosed = True
                        break
                except Exception as e:
                    self.get_logger().info(e)
                    self.exitThread = True
                    clientSocket.shutdown(socket.SHUT_RDWR)
                    clientSocket.close()
                    isClientClosed = True
                    break

        if (not isClientClosed):
            clientSocket.shutdown(socket.SHUT_RDWR)
            clientSocket.close()

    def finish(self):
        self.get_logger().info("Finishing leg_controller!\n")
        self.socket.shutdown(socket.SHUT_RDWR)
        self.socket.close()
        self.exitThread = True
        self.reqThread.join()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    legController = LegController()

    try:
        legController.run()
    except Exception as e:
        print("Got unhandled exception!\n")
        print(e)
    finally:
        legController.finish()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
