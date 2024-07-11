#!/usr/bin/env python3

# TEMPLATE CODE 
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
        self.simple_controller_enable = False
        
        self.declare_parameter('simple_controller_enable', False)
        
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
        self.timer_count = 0
        self.create_timer(1.0, self.update_parameters)
        self.create_timer(0.1, self.run)  

        self.get_logger().info("**************SimpleLegController initialized****************")

    def callback_position(self, msg):
        self.globalPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def joint_state_callback(self, msg):
        self.currPos = constrain_angle(np.array([*msg.position]))
        self.currVel = np.array([*msg.velocity])
        self.currTorq = np.array([*msg.effort])

        self.currPos = self.currPos[[4, 2, 0, 5, 3, 1]]     
        self.currVel = self.currVel[[4, 2, 0, 5, 3, 1]]
        self.currTorq = self.currTorq[[4, 2, 0, 5, 3, 1]]
        
        self.newdata = True  

    def imu_callback(self, msg):
        self.currPose = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.newdata = True  

    def update_parameters(self):
        
        self.cmd_tau = [-4.0, -4.0, -4.0, -4.0, -4.0, -4.0]
        self.cmd_pos = [0.0,0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0, 0.0, 0.00, 0.0]
        self.cmd_kd = [0.350, 0.350, 0.350, 0.350, 0.350, 0.350]
        self.cmd_kp = [0.65, 0.65, 0.65, 0.65, 0.65, 0.65]
        
        self.simple_controller_enable = self.get_parameter('simple_controller_enable').get_parameter_value().bool_value

        
        self.timer_count += 1

    def compute_controls(self):
        
        cmd_tau = np.array(self.cmd_tau)
        cmd_vel = np.array(self.cmd_vel)
        cmd_pos = np.array(self.cmd_pos)
        cmd_kp = np.array(self.cmd_kp)
        cmd_kd = np.array(self.cmd_kd)
        
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
            if self.simple_controller_enable:
                self.publisher.publish(torque)
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

    

