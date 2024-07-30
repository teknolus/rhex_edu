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

"""

NOTE: THE CODE IS STILL NOT SIMPLIFIED/ PARAMS ARE NOT TUNED PROPERLY


Terminal Commands to run the controller node:
To launch Gazebo:
    -launching gazebo: ros2 launch rhex_gazebo simple_start_sim.launch.py
To launch the controller node:
    -launching controller: ros2 launch rhex_control simple_start_controller_server.launch.py
To run the python file with all six modes (sitting, standing, walking1, walking2, turning right, turning left) that sends terminal commands to shell:
    -python3 /home/rhex/mnt/rhex_ws/src/rhex_control/scripts/buttons.py

VIDEO W ALL MODES DISPLAYED: https://drive.google.com/file/d/1duyPT5W0YsQ4HCWbGiJFwrvNbzAWRvWE/view?usp=sharing
"""




class TestRobot(Node):
    
    """
    A class for creating a ROS node that publishes torque commands to /effort_controller/commands.

    This node uses a PD controller to generate torque commands based on the given state and parameters.

    Attributes:
        cmd_kp (float np array): Proportional gain parameter for the PD controller.
        cmd_kd(float np array): derivative gain parameter for the PD controller.
        cmd_tau (float np array): desired torque for the PD controller.
        cmd_vel (float np array): desired velocity for the PD controller.
        cmd_pos (float np array): desired pose for the PD controller.
        
        state (int): State parameter indicating the current mode or task of the walker.
        simple_walker_enable (bool): Enables/disables the SimpleWalker node.

    Methods:
        __init__(self):
            Initializes the SimpleWalker node and sets up necessary variables, parameters, publishers and subscribers.

        callback_position(self, msg)
            Updates globalPos via node's subscription to /odom/robot_pos
            
        joint_state_callback(self, msg)
            Updates current pose, velocity, and torque via node's subscription to /joint_states
        
        imu_callback(self, msg)
            Updates current pose via node's subscription to /imu/data 

        print_joint_state(self)
            Prints current pose whenever it is called 
            
        compute_controls(self)
            Calculates the torque output of the PD controller given desired torque, velocity, and pose 
            
        def run(self)
            Includes the states, calls the compute_controls, and publishes commands to /effort_controller/commands 

    Usage:
        Create an instance of SimpleWalker, set parameters via ROS parameter server, and start the node to publish torque commands.
    """
    
    def __init__(self):
        super().__init__('test_robot')
        
        
        # declared parameters for communicating with the terminal 
        self.declare_parameter('velocity', 0)
        self.declare_parameter('state', 1)
        self.declare_parameter('simple_walker_enable', False)
        self.declare_parameter('cmd_tau', [0.0]*6)
        self.declare_parameter('cmd_vel', [0.0]*6)
        self.declare_parameter('cmd_pos', [0.0]*6)
        self.declare_parameter('cmd_kp', [10.0]*6)
        self.declare_parameter('cmd_kd', [0.35]*6)
        self.declare_parameter('period', 1.0)
        self.declare_parameter('delta_phi_s', 0.0)
        self.declare_parameter('delta_t_s', 0.0)
        
        
        
        # variables 
        self.newdata = False
        self.simple_walker_enable = False
        self.state = 1
        self.cmd_tau = [0.0] * 6 
        self.cmd_pos = [0.0] * 6
        self.cmd_vel = [0.0] * 6
        self.cmd_kp = [5.0] * 6
        self.cmd_kd = [0.35] * 6
        self.currPos = np.zeros(6)
        self.currVel = np.zeros(6)
        self.currTorq = np.zeros(6)
        self.currPose = np.zeros(4)
        self.globalPos = np.zeros(3)
        self.period = 1.0
        self.delta_phi_s = 0.0
        self.delta_t_s = 0.0
       
        
        # TOPICS
        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.Odometry_Subscriber_ = self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints_Subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subIMU_Subscriber_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # TIME SYNCHRONIZATIONS 
        self.start_time = self.get_clock().now() 
        # self.simulation_speedup = 1.0 (no longer using real time clock so not needed.)

        # run function publishes commands every 0.0025 second 
        self.create_timer(0.001, self.run)  
        self.get_logger().info("**************test_robot initialized****************")
        
           
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
        
    def print_joint_state(self):
        self.get_logger().info(f"current pose:{self.currPos}")


    # computes the output of the closed loop pd controller given desired pose, velocity, torque and kp, kd values 
    def compute_controls(self):
        cmd_tau = np.array(self.cmd_tau)
        cmd_vel = np.array(self.cmd_vel)
        cmd_pos = np.array(self.cmd_pos)
        cmd_kp = np.array(self.cmd_kp)
        cmd_kd = np.array(self.cmd_kd)
        curr_Pos = np.array(self.currPos)
        curr_Vel = np.array(self.currVel)
        curr_Torq = np.array(self.currTorq)
        posErr = (cmd_pos) - curr_Pos
        posErr = np.mod(posErr+np.pi, 2*np.pi) - np.pi
        velErr = (cmd_vel) - curr_Vel
        commTorque = np.multiply(cmd_kp, posErr) + np.multiply(cmd_kd, velErr) + cmd_tau
        np.clip(commTorque, -20, 20, out=commTorque)
        return list(commTorque[[2, 5, 1, 4, 0, 3]])
    
    def simple_sit (self): 

        for i, pos in enumerate(self.currPos):
            
                if (-math.pi  <= pos < 1.0):
                    self.cmd_pos[i] = -1.4
                    self.cmd_vel[i] = 0.0
                    
                elif (1.0 <= pos<= 3.14):
                    self.cmd_pos[i] = 2*math.pi + math.pi - 1.4
                    self.cmd_vel[i] = 0.5
                 
    def simple_stand (self):
                
        for i, pos in enumerate(self.currPos):
            if -3.14 < pos < -0.4:
                self.cmd_pos[i] = 0.0
                self.cmd_vel[i] = 0.2
                        
            elif -0.4 <= pos <= 0.4:
                self.cmd_pos[i] = 0.0
                self.cmd_vel[i] = 0.0
                        
            elif 0.4 <= pos < 1.0:
                self.cmd_pos[i] = 0.0
                self.cmd_vel[i] = -0.2
                    
            elif 1.0 <= pos < 3.14:
                self.cmd_pos[i] = 3.14
                self.cmd_vel[i] = 0.2
        
    def simple_walk (self):
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
         
        t_c = 1.0
        t_s = 0.5 
        t_f = t_c - t_s
        t_d = 0.01 
        phi_s = 0.6
                
        t = elapsed_time % t_c
                
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
    
    def simple_run (self):  
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
        
        t_c = 0.5
        t_s = 0.25
        t_f = t_c - t_s
        t_d = 0.01 
        phi_s = 0.6
                
        t = elapsed_time % t_c
                
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
    
    def simple_turn_right (self):
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
                
        t_c = 1.0
        t_s = 0.5
        t_f = t_c - t_s
        t_d = 0.0
        phi_s = 0.6
                
        t = elapsed_time % t_c
                
        v_s = phi_s / t_s
        v_f = (2*math.pi - phi_s)/(t_c - t_s)
                
                
        # RIGHT TRIPOD
        for i in [1, 3, 5]: 
            if (0 <= t < t_s):
                self.cmd_pos[i] = - v_s *t + phi_s/2
                self.cmd_vel[i] = - v_s
                            
            elif (t_s <= t < t_c):
                self.cmd_pos[i] = - v_f * (t - t_s) - phi_s/2
                self.cmd_vel[i] = - v_f
                            
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
         
    def simple_turn_left (self):
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
                 
                
        t_c = 1.0
        t_s = 0.5
        t_f = t_c - t_s
        t_d = 0.0
        phi_s = 0.6
                
        t = elapsed_time % t_c
                
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
                self.cmd_pos[i] = - v_f *(t - t_d) - phi_s/2
                self.cmd_vel[i] = - v_f
                            
            elif (t_d + t_f <= t < t_c):
                self.cmd_pos[i] = - v_s * (t- (t_d + t_f)) - (2* math.pi - phi_s/2)
                self.cmd_vel[i] = - v_s    
                        
            elif (0 <= t < t_d):
                self.cmd_pos[i] = - v_s * (t + t_s - t_d) - (2* math.pi -phi_s/2)
                self.cmd_vel[i] = - v_s
            

    def simple_walk_with_period (self):
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
         
        t_c = self.period
        t_s = self.period /2
        t_f = t_c - t_s
        t_d = 0.01 
        phi_s = 0.6
                
        t = elapsed_time % t_c
                
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
    
    def simple_walk_and_turn (self):
        current_time = self.get_clock().now()
        elapsed_duration = ((current_time- self.start_time)) 
        elapsed_time = elapsed_duration.nanoseconds /1e9
         
        t_c = 1.0
        t_s = 0.5
        t_d = 0.01 
        phi_s = 0.6
                
        t = elapsed_time % t_c

        delta_t_s = self.delta_t_s
        delta_phi_s = self.delta_phi_s

        t_s_l = t_s + delta_t_s
        t_s_r = t_s - delta_t_s

         
        t_f_r = t_c - t_s_r
        t_f_l = t_c - t_s_l

        phi_s_0_l = 0 + delta_phi_s
        phi_s_0_r = 0 - delta_phi_s
        
        
        v_s_r = phi_s / t_s_r
        v_f_r = (2*math.pi - phi_s)/(t_c - t_s_r) 
        
        v_s_l = phi_s / t_s_l 
        v_f_l = (2*math.pi - phi_s)/(t_c - t_s_l) 
                
                
        # RIGHT TRIPOD
        for i in [1, 3, 5]: 
            if (0 <= t < t_s_r):
                self.cmd_pos[i] = v_s_r *t - phi_s/2 + phi_s_0_r
                self.cmd_vel[i] = v_s_r
                            
            elif (t_s_r <= t < t_c):
                self.cmd_pos[i] = v_f_r * (t - t_s_r) + phi_s/2 + phi_s_0_r
                self.cmd_vel[i] = v_f_r
                            
        # LEFT TRIPOD 
        for i in [0, 2, 4]:
            if (t_d <= t < t_d + t_f_l):
                self.cmd_pos[i] = v_f_l *(t - t_d) + phi_s/2 + phi_s_0_l
                self.cmd_vel[i] = v_f_l
                            
            elif (t_d + t_f_l <= t < t_c):
                self.cmd_pos[i] = v_s_l * (t- (t_d + t_f_l)) + (2* math.pi - phi_s/2 + phi_s_0_l)
                self.cmd_vel[i] = v_s_l    
                        
            elif (0 <= t < t_d):
                self.cmd_pos[i] = v_s_l * (t + t_s_l - t_d) + (2* math.pi -phi_s /2 + phi_s_0_l)
                self.cmd_vel[i] = v_s_l
    
    def run(self):
        
        if (self.simple_walker_enable):
            
            # SIT
            if (self.state == 1):   
                self.simple_sit()
                       
            # SIAND
            if (self.state == 2):  
                self.simple_stand()
                       
            # WALK 
            if (self.state == 3): 
                self.simple_walk()  
  
            # RUN 
            if (self.state == 4):
                self.simple_run()
                
            # TURN RIGHT
            if (self.state == 5): 
                self.simple_turn_right()
                
            # TURN LEFT
            if (self.state == 6):
                self.simple_turn_left()

            # WALK WITH GIVEN PERIOD 
            if (self.state == 7):
                self.simple_walk_with_period()

            # WALK AND TURN RIGHT 
            if (self.state == 8):
                self.simple_walk_and_turn()
            
             
        self.simple_walker_enable = self.get_parameter('simple_walker_enable').get_parameter_value().bool_value
        self.state = self.get_parameter('state').get_parameter_value().integer_value
        self.cmd_tau = self.get_parameter('cmd_tau').get_parameter_value().double_array_value
        self.cmd_kd = self.get_parameter('cmd_kd').get_parameter_value().double_array_value
        self.cmd_kp = self.get_parameter('cmd_kp').get_parameter_value().double_array_value
        self.period = self.get_parameter('period').get_parameter_value().double_value 
        self.delta_phi_s = self.get_parameter('delta_phi_s').get_parameter_value().double_value
        self.delta_t_s = self.get_parameter('delta_t_s').get_parameter_value().double_value

        if self.newdata:
            torque = Float64MultiArray()
            torque.data = self.compute_controls()
            if (self.simple_walker_enable):
                self.publisher.publish(torque)
            self.newdata = False
        
def main (args = None):
    rclpy.init(args = args)
    node = TestRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
    
if __name__ == '__main__':
    main()
    
