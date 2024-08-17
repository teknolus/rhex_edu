#!/usr/bin/env python3

import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from control_msgs.action import FollowJointTrajectory
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import time 
import math


class SimpleWalker(Node):
    
    def __init__(self):
        super().__init__('walker')
        
        
        # declared parameters for communicating with the terminal 
        self.declare_parameter('state', 3)
        self.declare_parameter('walker_enable', True)
        self.declare_parameter('cmd_tau', [0.0]*6)
        self.declare_parameter('cmd_vel', [0.0]*6)
        self.declare_parameter('cmd_pos', [0.0]*6)
        self.declare_parameter('cmd_kp', [0.0]*6)
        self.declare_parameter('cmd_kd', [0.0]*6)
        self.declare_parameter('delta_t_s', 0.0)
                
        
        
        # variables 
        self.walker_enable = False
        self.state = 1
        self.cmd_tau = [0.0] * 6 
        self.cmd_pos = [0.0] * 6
        self.cmd_vel = [0.0] * 6
        self.cmd_kp = [0.0] * 6
        self.cmd_kd = [0.0] * 6
        self.currPos = np.zeros(6)
        self.currVel = np.zeros(6)
        self.currTorq = np.zeros(6)
        self.currPose = np.zeros(4)
        self.globalPos = np.zeros(3)
        self.counter = 0 
        self.delta_t_s = 0.0

        #sitting and standing params 
        self.start_sitting = [True] * 6 
        self.sit_start_time = [0.0] * 6
        self.start_standing = [True] * 6
        self.stand_start_time = [0.0] * 6
        self.a = [0.0] * 6
        self.b= [0.0] * 6
        
        # TOPICS
        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.Odometry_Subscriber_ = self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints_Subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subIMU_Subscriber_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # TIME
        self.start_time = self.get_clock().now() 
        
        # OPTIMIZATION 
        self.command_torque = [0.0] * 6
        self.command_torque_subscriber = self.create_subscription(Float64MultiArray, '/command_torque', self.callback_command_torque, 10)
        self.command_position_publisher = self.create_publisher(Float64MultiArray, 'command_position', 10)
        self.command_velocity_publisher = self.create_publisher(Float64MultiArray, 'command_velocity', 10)
        self.current_position_publisher = self.create_publisher(Float64MultiArray, 'current_position', 10)
        self.current_velocity_publisher = self.create_publisher(Float64MultiArray, 'current_velocity', 10)
        
        self.create_timer(0.001, self.run)  
        self.get_logger().info("**************walker initialized****************")    

    def get_sim_time(self):
        sim_time = self.get_clock().now().to_msg()
        self.get_logger().info(f"Simulation time: {sim_time.sec}.{sim_time.nanosec}")
        self.get_logger().info(f"current pose:{self.currPos}")

    def callback_position(self, msg):
        self.globalPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def joint_state_callback(self, msg):
        self.currPos = constrain_angle(np.array([*msg.position]))
        self.currVel = np.array([*msg.velocity])
        self.currTorq = np.array([*msg.effort])
        self.currPos = self.currPos[[4, 2, 0, 5, 3, 1]]     
        self.currVel = self.currVel[[4, 2, 0, 5, 3, 1]] 
        self.currTorq = self.currTorq[[4, 2, 0, 5, 3, 1]]

    def imu_callback(self, msg):
        self.currPose = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.newdata = True 
        
    def callback_command_torque(self, msg):
        self.command_torque = msg.data   
    
    
    def print_joint_state(self):
        if self.counter % 5 == 0:
            self.get_logger().info(f"current pose:{self.currPos}")
            self.get_logger().info(f"current velocity:{self.currVel}")
            self.get_logger().info(f"current torque: {self.currTorq}")
        self.counter += 1
        if self.counter % 5 == 0:
            self.get_logger().info(f"current pose:{self.currPos}")
            self.get_logger().info(f"current velocity:{self.currVel}")
            self.get_logger().info(f"current torque: {self.currTorq}")
        self.counter += 1

    def publish_controls(self):
        published_torque = np.array(self.command_torque)
        published_torque = list(published_torque[[2, 5, 1, 4, 0, 3]])
        return published_torque

    def publish_command_position(self):
        cmdpos = np.array(self.cmd_pos)
        position = Float64MultiArray()
        position.data = list(cmdpos)
        return position   
    
    def publish_command_velocity(self):
        cmdvel = np.array(self.cmd_vel)
        velocity = Float64MultiArray()
        velocity.data = list(cmdvel)
        return velocity   
    
    def publish_current_position(self):
        currPos = np.array(self.currPos)
        position= Float64MultiArray()
        position.data = list(currPos)
        return position
    
    def publish_current_velocity(self):
        currVel = np.array(self.currVel)
        velocity= Float64MultiArray()
        velocity.data = list(currVel)
        return velocity
    
    def simple_sit (self): 
        
        t = [0.0] * 6 
        t_c = 4.0   
                       
        for i in range(6): 
            sitting_point = - 1.9
            
            if -0.2 + sitting_point < self.currPos[i] < 0.2 + sitting_point:
                self.cmd_pos[i] = sitting_point
                self.cmd_vel[i] = 0.0
                self.start_sitting [i] = True 
            else: 
                elapsed_duration = self.get_clock().now() - self.sit_start_time[i]
                t [i] = (elapsed_duration.nanoseconds /1e9)
            
            if t[i] < t_c:
                if self.start_sitting [i]: 
                    self.b [i] = self.currPos [i] 
                    self.a [i] = (sitting_point -self.currPos [i])  / t_c 
                    self.start_sitting [i] = False 
  
                self.cmd_pos[i] = self.a[i] * t [i] + self.b[i] 
                self.cmd_vel[i] = self.a[i]
            else: 
                self.cmd_pos[i] = sitting_point
                self.cmd_vel[i] = 0.0
                self.start_sitting [i] = True 
                           
    def simple_stand (self):
        
        t = [0.0] *6 
        t_c = 4.0   
               
        for i in range(6): 
          
            if -0.1< self.currPos[i] < 0.1:
                self.cmd_pos[i] = 0.0
                self.cmd_vel[i] = 0.0
                self.start_standing [i] = True 
            else: 
                elapsed_duration = self.get_clock().now() - self.stand_start_time[i]
                t [i] = (elapsed_duration.nanoseconds /1e9)
            
            if t[i] < t_c:
                if self.start_standing [i]: 
                    self.b [i] = self.currPos [i] 
                    self.a [i] = -self.currPos [i] / t_c 
                    self.start_standing [i] = False 
                    
                self.cmd_pos[i] = self.a[i] * t [i] + self.b[i] 
                self.cmd_vel[i] = self.a[i]
            else: 
                self.cmd_pos[i] = 0.0
                self.cmd_vel[i] = 0.0
                self.start_standing [i] = True 

    def simple_walk (self):
        
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
         
        t_c = 1.0
        t_s = 0.5 
        t_c = 1.0
        t_s = 0.5 
        t_f = t_c - t_s
        t_d = 0.01 
        phi_s = 0.6
                
        t = elapsed_time % t_c
        v_s = phi_s / t_s 
        v_f = (2*math.pi - phi_s)/(t_c - t_s) 
        v_s = phi_s / t_s 
        v_f = (2*math.pi - phi_s)/(t_c - t_s) 
          
        # RIGHT TRIPOD
        for i in [1, 3, 5]: 
            if (0 <= t < t_s):
                self.cmd_pos[i] = v_s *t - phi_s/2
                self.cmd_vel[i] = v_s
                            
            elif (t_s <= t < t_c):
                self.cmd_pos[i] = v_f * (t - t_s) + phi_s/2
                self.cmd_vel[i] = v_f
                            
        # LEFT TRIPOD 
        for i in [0, 2, 4]:
            if (t_d <= t < t_d + t_f):
                self.cmd_pos[i] = v_f *(t - t_d) + phi_s/2
                self.cmd_vel[i] = v_f
                            
            elif (t_d + t_f <= t < t_c):
                self.cmd_pos[i] = v_s * (t- (t_d + t_f)) + (2* math.pi - phi_s/2)
                self.cmd_vel[i] = v_s    
                        
            elif (0 <= t < t_d):
                self.cmd_pos[i] = v_s * (t + t_s - t_d) + (2* math.pi -phi_s/2)
                self.cmd_vel[i] = v_s
    
    def constant_pos (self):
        self.cmd_vel = np.zeros(6)
        self.cmd_pos = np.zeros(6)
        
    def run(self):

        if (self.walker_enable):
            
            #SIT
            if (self.state == 1):   
                for i in range (6):
                    if (self.start_sitting[i]):
                        self.sit_start_time[i] = self.get_clock().now()
                self.simple_sit()

            # SIAND
            if (self.state == 2):  

                for i in range (6):
                    if (self.start_standing[i]) :
                        self.stand_start_time[i] = self.get_clock().now()
                    else: 
                        pass

                self.simple_stand()
                       
            # WALK 
            if (self.state == 3): 
                self.simple_walk()  
             
            # constant 
            if (self.state == 10):
                self.constant_pos()     
        self.walker_enable = self.get_parameter('walker_enable').get_parameter_value().bool_value
        self.state = self.get_parameter('state').get_parameter_value().integer_value

        
        torque = Float64MultiArray()
        torque.data = self.publish_controls()
        if (self.walker_enable):
            self.publisher.publish(torque)
        
        cmdpos = self.publish_command_position()
        if (self.walker_enable):
            self.command_position_publisher.publish(cmdpos)  
        
        cmdvel = self.publish_command_velocity()
        if (self.walker_enable):
            self.command_velocity_publisher.publish(cmdvel)  
            
        currentpos = self.publish_current_position()
        if (self.walker_enable):
            self.current_position_publisher.publish(currentpos)  
        
        currentvel = self.publish_current_velocity()
        if (self.walker_enable):
            self.current_velocity_publisher.publish(currentvel)  
        
        
def main (args = None):
    rclpy.init(args = args)
    node = SimpleWalker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
          
          
if __name__ == '__main__':
    main()

    
# note that the current model sends the commands with a 1 ms (real time) delay so sent torques to rhex are delayed by 1ms.
