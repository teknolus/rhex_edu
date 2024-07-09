#!/usr/bin/env python3

import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry


class SimpleLegController(Node):
    def __init__(self):
        super().__init__('simple_leg_controller')

        # Use lists for parameter defaults
        self.declare_parameter('cmd_tau', [0.0]*6)
        self.declare_parameter('cmd_vel', [0.0]*6)
        self.declare_parameter('cmd_pos', [0.0]*6)
        self.declare_parameter('cmd_kp', [0.0]*6)
        self.declare_parameter('cmd_kd', [0.0]*6)

        self.cmd_tau = self.get_parameter('cmd_tau').get_parameter_value().double_array_value
        self.cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().double_array_value
        self.cmd_pos = self.get_parameter('cmd_pos').get_parameter_value().double_array_value
        self.cmd_kp = self.get_parameter('cmd_kp').get_parameter_value().double_array_value
        self.cmd_kd = self.get_parameter('cmd_kd').get_parameter_value().double_array_value

        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        self.Odometry_Subscriber_ = self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints_Subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subIMU_Subscriber_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.currPos = np.zeros(6)
        self.currVel = np.zeros(6)
        self.currTorq = np.zeros(6)
        self.currPose = np.zeros(4)
        self.globalPos = np.zeros(3)
        
        self.newdata = False
        self.create_timer(1.0, self.update_parameters)
        self.create_timer(0.1, self.run)  

        self.get_logger().info("**************SimpleLegController initialized****************")

    def callback_position(self, msg):
        self.globalPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.get_logger().info(f"Position: {self.globalPos}")

    def joint_state_callback(self, msg):
        self.currPos = constrain_angle(np.array([*msg.position]))
        self.currVel = np.array([*msg.velocity])
        self.currTorq = np.array([*msg.effort])

        # Change the leg order so that it matches with the RosNet configuration
        self.currPos = self.currPos[[4, 2, 0, 5, 3, 1]]     
        self.currVel = self.currVel[[4, 2, 0, 5, 3, 1]]
        self.currTorq = self.currTorq[[4, 2, 0, 5, 3, 1]]
        
        self.newdata = True  # Set newdata to True when new data is received
        self.get_logger().info("**************Joint State Updated****************")

    def imu_callback(self, msg):
        self.currPose = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.newdata = True  # Set newdata to True when new data is received
        self.get_logger().info("**************IMU State Updated****************")

    def update_parameters(self):
        # Update parameters from the parameter server
        self.cmd_tau = self.get_parameter('cmd_tau').get_parameter_value().double_array_value
        self.cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().double_array_value
        self.cmd_pos = self.get_parameter('cmd_pos').get_parameter_value().double_array_value
        self.cmd_kp = self.get_parameter('cmd_kp').get_parameter_value().double_array_value
        self.cmd_kd = self.get_parameter('cmd_kd').get_parameter_value().double_array_value

    def compute_controls(self):
        
        cmd_tau = np.array(self.get_parameter('cmd_tau').get_parameter_value().double_array_value)
        cmd_vel = np.array(self.get_parameter('cmd_vel').get_parameter_value().double_array_value)
        cmd_pos = np.array(self.get_parameter('cmd_pos').get_parameter_value().double_array_value)
        cmd_kp = np.array(self.get_parameter('cmd_kp').get_parameter_value().double_array_value)
        cmd_kd = np.array(self.get_parameter('cmd_kd').get_parameter_value().double_array_value)
        
            # Ensure currPos, currVel, currTorq are numpy arrays
        curr_Pos = np.array(self.currPos)
        curr_Vel = np.array(self.currVel)
        curr_Torq = np.array(self.currTorq)
    
        posErr = (-cmd_pos) - curr_Pos
        posErr = np.mod(posErr+np.pi, 2*np.pi) - np.pi
        velErr = (-cmd_vel) - curr_Vel
        commTorque = np.multiply(cmd_kp, posErr) + np.multiply(cmd_kd, velErr) - cmd_tau
        np.clip(commTorque, -20, 20, out=commTorque)
        return list(commTorque[[2, 5, 1, 4, 0, 3]])
    
    def run(self):
        if self.newdata:
            torque = Float64MultiArray()
            torque.data = self.compute_controls()
            self.publisher.publish(torque)
            self.get_logger().info("-------------------TORQUE PUBLISHED----------------")
            self.newdata = False

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLegController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
