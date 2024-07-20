#!/usr/bin/env python3

import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import time 
import math

"""
Terminal Commands to run the controller node:
To launch Gazebo:
    -launching gazebo: ros2 launch rhex_gazebo simple_start_sim.launch.py
To launch the controller node:
    -launching controller: ros2 launch rhex_control simple_start_controller_server.launch.py
To run the python file with all six modes (sitting, standing, walking1, walking2, turning right, turning left, dancing) that sends terminal commands to shell:
    -python3 /home/rhex/mnt/rhex_ws/src/rhex_control/scripts/buttons.py

VIDEO W ALL MODES DISPLAYED: https://drive.google.com/file/d/1duyPT5W0YsQ4HCWbGiJFwrvNbzAWRvWE/view?usp=sharing
"""




class SimpleWalker(Node):
    
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
        super().__init__('simple_walker')
        
        
        # declared parameters for communicating with the terminal 
        self.declare_parameter('state', 1)
        self.declare_parameter('simple_walker_enable', False)
        self.declare_parameter('cmd_tau', [0.0]*6)
        self.declare_parameter('cmd_vel', [0.0]*6)
        self.declare_parameter('cmd_pos', [0.0]*6)
        self.declare_parameter('cmd_kp', [0.0]*6)
        self.declare_parameter('cmd_kd', [0.0]*6)
        
        
        
        # variables 
        
        self.newdata = False
        
        self.start_time = time.time()
        self.simulation_speedup = 1.516  
        
        self.simple_walker_enable = False
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
        
       
        
        # TOPICS
        
        # node publishes torque commands to /effort_controller/commands topic
        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        # node subscribes to /odom/robot/pos, /joint_states, /imu/data topics (currently only /joint_states data is utilized.)
        self.Odometry_Subscriber_ = self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints_Subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subIMU_Subscriber_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        
        # run function publishes commands every 0.0025 second 
        self.create_timer(0.0025, self.run)  
        
        self.get_logger().info("**************SimpleWalker initialized****************")
        

           
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
    
    # checks if node is enabled, then based on the state, chooses corresponding cmd_tau, cmd_vel, cmd_pos, cmd_kp, cmd_kd values, 
    # computes the closed loop torque output and publishes it to the /effort_controller/commands 
    
    def run(self):
        
        if (self.simple_walker_enable):
            
            # SIT
            if (self.state == 1):   
                self.cmd_kp = [3.75, 3.75, 3.75, 3.75, 3.75, 3.75]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35] 
                for i, pos in enumerate(self.currPos):
                    if (-math.pi  <= pos < -1.0):
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -1.3
                        self.cmd_vel[i] = -0.
                    elif (-1.0 <= pos < 1.0):
                        self.cmd_tau[i] = -1.0
                        self.cmd_pos[i] = -1.3
                        self.cmd_vel[i] = -0.5
                    elif (1.0 <= pos<= 3.14):
                        self.cmd_tau[i] = 1.0
                        self.cmd_pos[i] = 2*math.pi -1.3
                        self.cmd_vel[i] = 0.5
            # SIAND
            if (self.state == 2):  
                  
                self.cmd_kp = [3.75, 3.75, 3.75, 3.75, 3.75, 3.75]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
                
                for i, pos in enumerate(self.currPos):
                    if -3.14 < pos < -0.4:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.5
                        
                    elif -0.4 <= pos <= 0.4:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                        
                    elif 0.4 <= pos < 1.0:
                        self.cmd_tau[i] = -2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = -0.5
                        
                    elif 1.0 <= pos < 3.14:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 0.5
            # WALK MODE 1_ SLOWER
            if (self.state == 3):   
                elapsed_time = ((time.time() - self.start_time)) * self.simulation_speedup
                t_c = 2.0
                t_s = 1.0 
                
                self.cmd_kp = [4.75, 4.75, 4.75, 4.75, 4.75, 4.75]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
                
                t_d = 0.01 # assumption: t_d < t_s /8
                phi_s = 0.6
                
                t = elapsed_time % t_c
                
                ##################RIGHT TRIPOD #################
                #### [-phi_s/2, 0]####
                if 0 <= t < (t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /8) <= t < (t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /4) <= t < (3* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/8
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /8) <= t < (t_s /2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/t_s)
                 
                #### [0, phi_s/2] ####
                     
                elif t_s/2 <= t < (5* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/t_s)
                        
                elif (5* t_s /8) <= t < (3* t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /4) <= t < (7* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3*phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (7* t_s /8) <= t < (t_s):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] ####
                
                elif t_s <= t < (t_s + (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/4
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + (t_c -t_s)/4) <= t < (t_s + 3* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 3* (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                 
                #### [-pi, -phi_s/2]
                elif (t_s + (t_c -t_s)/2) <= t < (t_s + 5* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+(2*math.pi - phi_s/2))/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 5* (t_c -t_s)/8) <= t < (t_s + 3* (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + 3* (t_c -t_s)/4)<= t < (t_s + 7* (t_c -t_s)/8) :
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((2*math.pi - phi_s/2)+ (math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif ((t_s + 7* (t_c -t_s)/8) + 3* (t_c -t_s)/4)<= t < t_c:
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                ##################LEFT TRIPOD #################
                #### [0, phi_s/2] #### 
                if (t_d + t_c - t_s + (t_s)/2) <= t < (t_d +t_c - t_s  + 5* (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + 5* (t_s)/8) <= t < (t_d + t_c - t_s  + 3* (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + 3* (t_s)/4) <= t < (t_d + t_c - t_s + 7* (t_s)/8) :
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3* phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif 0 <= t < t_d or (t_d + t_c - t_s + 7* (t_s)/8) <= t < t_c:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] #####
                elif t_d <= t < (t_d + (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2 + (math.pi+phi_s/2)/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/4) <= t < (t_d + 3* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_d + 3* (t_c - t_s)/8) <= t < (t_d + t_s/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                #### [-pi, -phi_s/2]
                elif (t_d + (t_c - t_s)/2) <= t < (t_d + 5* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi +(math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)  
                        
                elif  (t_d + 5* (t_c - t_s)/8) <= t < (t_d + 3* (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s) 
                
                elif (t_d+ 3* (t_c - t_s)/4) <= t < (t_d+ 7* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((math.pi+(2*math.pi - phi_s/2))/2 + (2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif  (t_d+ 7* (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                
                #### [-phi_s/2, 0]
                elif (t_d + (t_c - t_s)) <= t < (t_d + t_c - t_s  + (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + (t_s)/8) <= t < (t_d + t_c - t_s  + (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + (t_s)/4) <= t < (t_d + t_c - t_s + 3*(t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s /8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s + 3*(t_s)/8) <= t < (t_d + t_c - t_s + (t_s)/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/ t_s)
                
             # WALK MODE 2. 
            # WALK MODE 2_ FASTER
            if (self.state == 4):   
                elapsed_time = ((time.time() - self.start_time)) * self.simulation_speedup
                t_c = 1.4
                t_s = 0.7
                
                self.cmd_kp = [5.75, 5.75, 5.75, 5.75, 5.75, 5.75]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
                
                t_d = 0.0 # assumption: t_d < t_s /8
                phi_s = 0.6
                
                t = elapsed_time % t_c

                ##################RIGHT TRIPOD #################
                #### [-phi_s/2, 0]####
                if 0 <= t < (t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /8) <= t < (t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /4) <= t < (3* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/8
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /8) <= t < (t_s /2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/t_s)
                 
                #### [0, phi_s/2] ####
                     
                elif t_s/2 <= t < (5* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/t_s)
                        
                elif (5* t_s /8) <= t < (3* t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /4) <= t < (7* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3*phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (7* t_s /8) <= t < (t_s):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] ####
                
                elif t_s <= t < (t_s + (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/4
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + (t_c -t_s)/4) <= t < (t_s + 3* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 3* (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                 
                #### [-pi, -phi_s/2]
                elif (t_s + (t_c -t_s)/2) <= t < (t_s + 5* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+(2*math.pi - phi_s/2))/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 5* (t_c -t_s)/8) <= t < (t_s + 3* (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + 3* (t_c -t_s)/4)<= t < (t_s + 7* (t_c -t_s)/8) :
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((2*math.pi - phi_s/2)+ (math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif ((t_s + 7* (t_c -t_s)/8) + 3* (t_c -t_s)/4)<= t < t_c:
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                ##################LEFT TRIPOD #################
                #### [0, phi_s/2] #### 
                if (t_d + t_c - t_s + (t_s)/2) <= t < (t_d +t_c - t_s  + 5* (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + 5* (t_s)/8) <= t < (t_d + t_c - t_s  + 3* (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + 3* (t_s)/4) <= t < (t_d + t_c - t_s + 7* (t_s)/8) :
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3* phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif 0 <= t < t_d or (t_d + t_c - t_s + 7* (t_s)/8) <= t < t_c:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] #####
                elif t_d <= t < (t_d + (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2 + (math.pi+phi_s/2)/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/4) <= t < (t_d + 3* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_d + 3* (t_c - t_s)/8) <= t < (t_d + t_s/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                #### [-pi, -phi_s/2]
                elif (t_d + (t_c - t_s)/2) <= t < (t_d + 5* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi +(math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)  
                        
                elif  (t_d + 5* (t_c - t_s)/8) <= t < (t_d + 3* (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s) 
                
                elif (t_d+ 3* (t_c - t_s)/4) <= t < (t_d+ 7* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((math.pi+(2*math.pi - phi_s/2))/2 + (2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif  (t_d+ 7* (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                
                #### [-phi_s/2, 0]
                elif (t_d + (t_c - t_s)) <= t < (t_d + t_c - t_s  + (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + (t_s)/8) <= t < (t_d + t_c - t_s  + (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + (t_s)/4) <= t < (t_d + t_c - t_s + 3*(t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s /8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s + 3*(t_s)/8) <= t < (t_d + t_c - t_s + (t_s)/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/ t_s)
            #  TURN RIGHT
            if (self.state == 5):   
                elapsed_time = ((time.time() - self.start_time)) * self.simulation_speedup
                t_c = 2.0
                t_s = 1.0
                
                self.cmd_kp = [10.00, 10.0, 10.00, 10.0 , 10.00, 10.0]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
                
                t_d = 0.2 # assumption: t_d < t_s /8
                phi_s = 0.7
                
                t = elapsed_time % t_c
                ##################RIGHT TRIPOD #################
                #### [phi_s/2, 0]####
                if 0 <= t < (t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 3* phi_s/8
                        self.cmd_vel[i] = -(phi_s/ t_s)
                        
                elif (t_s /8) <= t < (t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = phi_s/4
                        self.cmd_vel[i] = -(phi_s/ t_s)
                        
                elif (t_s /4) <= t < (3* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = phi_s/8
                        self.cmd_vel[i] = -(phi_s/t_s)
                
                elif (3* t_s /8) <= t < (t_s /2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = -(phi_s/t_s)
                 
                #### [0, -phi_s/2] ####
                     
                elif t_s/2 <= t < (5* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -(phi_s/8)
                        self.cmd_vel[i] = -(phi_s/t_s)
                        
                elif (5* t_s /8) <= t < (3* t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -(phi_s/4)
                        self.cmd_vel[i] = -(phi_s/t_s)
                
                elif (3* t_s /4) <= t < (7* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -(3*phi_s/8)
                        self.cmd_vel[i] = -(phi_s/ t_s)
                
                elif (7* t_s /8) <= t < (t_s):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -(phi_s/2)
                        self.cmd_vel[i] = -(phi_s/ t_s)
                
                #### [-phi_s/2, -pi] ####
                
                elif t_s <= t < (t_s + (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi-phi_s/2)/4
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi-phi_s/2)/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + (t_c -t_s)/4) <= t < (t_s + 3* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi + (-math.pi-phi_s/2)/2)/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 3* (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -math.pi
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                 
                #### [pi, phi_s/2]
                elif (t_s + (t_c -t_s)/2) <= t < (t_s + 5* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi + (-math.pi+(-2*math.pi + phi_s/2))/2) /2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 5* (t_c -t_s)/8) <= t < (t_s + 3* (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi+(-2*math.pi + phi_s/2))/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + 3* (t_c -t_s)/4)<= t < (t_s + 7* (t_c -t_s)/8) :
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((-2*math.pi + phi_s/2)+ (-math.pi+(-2*math.pi + phi_s/2))/2)/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                elif ((t_s + 7* (t_c -t_s)/8) + 3* (t_c -t_s)/4)<= t < t_c:
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-2*math.pi + phi_s/2)
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                ##################LEFT TRIPOD #################
                #### [0, phi_s/2] #### 
                if (t_d + t_c - t_s + (t_s)/2) <= t < (t_d +t_c - t_s  + 5* (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + 5* (t_s)/8) <= t < (t_d + t_c - t_s  + 3* (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + 3* (t_s)/4) <= t < (t_d + t_c - t_s + 7* (t_s)/8) :
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3* phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif 0 <= t < t_d or (t_d + t_c - t_s + 7* (t_s)/8) <= t < t_c:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] #####
                elif t_d <= t < (t_d + (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2 + (math.pi+phi_s/2)/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/4) <= t < (t_d + 3* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_d + 3* (t_c - t_s)/8) <= t < (t_d + t_s/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                #### [-pi, -phi_s/2]
                elif (t_d + (t_c - t_s)/2) <= t < (t_d + 5* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi +(math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)  
                        
                elif  (t_d + 5* (t_c - t_s)/8) <= t < (t_d + 3* (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s) 
                
                elif (t_d+ 3* (t_c - t_s)/4) <= t < (t_d+ 7* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((math.pi+(2*math.pi - phi_s/2))/2 + (2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif  (t_d+ 7* (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                
                #### [-phi_s/2, 0]
                elif (t_d + (t_c - t_s)) <= t < (t_d + t_c - t_s  + (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + (t_s)/8) <= t < (t_d + t_c - t_s  + (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + (t_s)/4) <= t < (t_d + t_c - t_s + 3*(t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s /8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_d + t_c - t_s + 3*(t_s)/8) <= t < (t_d + t_c - t_s + (t_s)/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/ t_s)
            # TURN LEFT
            if (self.state == 6):   
                elapsed_time = ((time.time() - self.start_time)) * self.simulation_speedup
                t_c = 2.0
                t_s = 1.0 
                
                self.cmd_kp = [10.00, 10.0, 10.00, 10.0 , 10.00, 10.0]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
                
                t_d = 0.1 # assumption: t_d < t_s /8
                phi_s = 0.7
                
                t = elapsed_time % t_c

                #########################RIGHT TRIPOD ######################################
                #### [-phi_s/2, 0]####
                if 0 <= t < (t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /8) <= t < (t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /4) <= t < (3* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/8
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /8) <= t < (t_s /2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/t_s)
                 
                #### [0, phi_s/2] ####
                     
                elif t_s/2 <= t < (5* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/t_s)
                        
                elif (5* t_s /8) <= t < (3* t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /4) <= t < (7* t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3*phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (7* t_s /8) <= t < (t_s):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] ####
                
                elif t_s <= t < (t_s + (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/4
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + (t_c -t_s)/4) <= t < (t_s + 3* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 3* (t_c -t_s)/8) <= t < (t_s + (t_c -t_s)/2):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                 
                #### [-pi, -phi_s/2]
                elif (t_s + (t_c -t_s)/2) <= t < (t_s + 5* (t_c -t_s)/8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+(2*math.pi - phi_s/2))/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 5* (t_c -t_s)/8) <= t < (t_s + 3* (t_c -t_s)/4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + 3* (t_c -t_s)/4)<= t < (t_s + 7* (t_c -t_s)/8) :
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((2*math.pi - phi_s/2)+ (math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif ((t_s + 7* (t_c -t_s)/8) + 3* (t_c -t_s)/4)<= t < t_c:
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                #########################LEFT TRIPOD ####################
                #### [0, -phi_s/2] #### 
                if (t_d + t_c - t_s + (t_s)/2) <= t < (t_d +t_c - t_s  + 5* (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-phi_s/8)
                        self.cmd_vel[i] = (-phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + 5* (t_s)/8) <= t < (t_d + t_c - t_s  + 3* (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -0.0
                        self.cmd_pos[i] = (-phi_s/4)
                        self.cmd_vel[i] = (-phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + 3* (t_s)/4) <= t < (t_d + t_c - t_s + 7* (t_s)/8) :
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-3* phi_s/8)
                        self.cmd_vel[i] = (-phi_s/ t_s)
                        
                elif 0 <= t < t_d or (t_d + t_c - t_s + 7* (t_s)/8) <= t < t_c:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-phi_s/2)
                        self.cmd_vel[i] = (-phi_s/ t_s)
                
                #### [-phi_s/2, -pi] #####
                elif t_d <= t < (t_d + (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-phi_s/2 + (-math.pi-phi_s/2)/2) /2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi-phi_s/2)/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_d + (t_c - t_s)/4) <= t < (t_d + 3* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi + (-math.pi-phi_s/2)/2)/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_d + 3* (t_c - t_s)/8) <= t < (t_d + t_s/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -math.pi
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                #### [pi, phi_s/2]
                elif (t_d + (t_c - t_s)/2) <= t < (t_d + 5* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi +(-math.pi+(-2*math.pi + phi_s/2))/2)/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)  
                        
                elif  (t_d + 5* (t_c - t_s)/8) <= t < (t_d + 3* (t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-math.pi+(-2*math.pi + phi_s/2))/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s) 
                
                elif (t_d+ 3* (t_c - t_s)/4) <= t < (t_d+ 7* (t_c - t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((-math.pi+(-2*math.pi + phi_s/2))/2 + (-2*math.pi + phi_s/2))/2
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                elif  (t_d+ 7* (t_c - t_s)/8) <= t < (t_d + (t_c - t_s)):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (-2*math.pi + phi_s/2)
                        self.cmd_vel[i] = -(2* math.pi - phi_s)/(t_c - t_s)
                
                
                #### [phi_s/2, 0]
                elif (t_d + (t_c - t_s)) <= t < (t_d + t_c - t_s  + (t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 3* phi_s/8
                        self.cmd_vel[i] = -(phi_s/ t_s)
                        
                elif (t_d + t_c - t_s  + (t_s)/8) <= t < (t_d + t_c - t_s  + (t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = phi_s/4
                        self.cmd_vel[i] = -(phi_s/ t_s)
                
                elif (t_d + t_c - t_s  + (t_s)/4) <= t < (t_d + t_c - t_s + 3*(t_s)/8):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = phi_s /8
                        self.cmd_vel[i] = -(phi_s/ t_s)
                        
                elif (t_d + t_c - t_s + 3*(t_s)/8) <= t < (t_d + t_c - t_s + (t_s)/2):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = -(phi_s/ t_s)
            
            # DANCEEEEEEEEEEEEEEEEEEEEEEE
            if (self.state == 7):   
                elapsed_time = ((time.time() - self.start_time)) * self.simulation_speedup
                t_c = 1.0
                t_s = 0.5 
                
                self.cmd_kp = [20.75, 20.75, 20.75, 20.75, 20.75, 20.75]
                self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
                
                
                phi_s = 0.6
                
                t = elapsed_time % (t_c + 2.0)
                
                ##################RIGHT TRIPOD #################
                #### [-phi_s/2, 0]####
                if 0 <= t < (t_s /8):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_s /8) <= t < (t_s /4):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                        
                        
                elif (t_s /4 ) <= t < (3* t_s /8 ):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/8
                        self.cmd_vel[i] = (phi_s/t_s)
                
                elif (3* t_s /8 ) <= t < (t_s /2 ):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/t_s)
                 
                 #### DANCE BREAK ####################################################################################
                elif (t_s /2) <= t < (t_s /2 +1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                #### DANCE BREAK ####################################################################################
                
                #### [0, phi_s/2] ####
                     
                elif (t_s/2 + 1.0) <= t < (5* t_s /8 + 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/t_s)
                        
                elif (5* t_s /8 + 1.0) <= t < (3* t_s /4 + 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/t_s)
                

                elif (3* t_s /4 + 1.0) <= t < (7* t_s /8 + 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3*phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (7* t_s /8 + 1.0) <= t < (t_s + 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] ####
                
                elif (t_s+ 1.0) <= t < (t_s + (t_c -t_s)/8 + 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/4
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + (t_c -t_s)/8+ 1.0) <= t < (t_s + (t_c -t_s)/4+ 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + (t_c -t_s)/4+ 1.0) <= t < (t_s + 3* (t_c -t_s)/8+ 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 3* (t_c -t_s)/8 + 1.0) <= t < (t_s + (t_c -t_s)/2+ 1.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                
                ######################### DANCE BREAKKKKKKKKKKKKKKKKKKKKKK #############################################
                elif (t_s + (t_c -t_s)/2 + 1.0)  <= t < (t_s + (t_c -t_s)/2+ 1.5):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi/2)
                        self.cmd_vel[i] =  0.0
                        
                elif (t_s + (t_c -t_s)/2+ 1.5)<= t < (t_s + (t_c -t_s)/2+ 2.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi)
                        self.cmd_vel[i] = 0.0
                       
                ######################### DANCE BREAKKKKKKKKKKKKKKKKKKKKKK #############################################
                #### [-pi, -phi_s/2]
                elif (t_s + (t_c -t_s)/2 + 2.0) <= t < (t_s + 5* (t_c -t_s)/8 + 2.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+(2*math.pi - phi_s/2))/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (t_s + 5* (t_c -t_s)/8 + 2.0) <= t < (t_s + 3* (t_c -t_s)/4 + 2.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif (t_s + 3* (t_c -t_s)/4 + 2.0)<= t < (t_s + 7* (t_c -t_s)/8 + 2.0) :
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((2*math.pi - phi_s/2)+ (math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif ((t_s + 7* (t_c -t_s)/8 + 2.0) + 3* (t_c -t_s)/4)<= t < (t_c + 2.0):
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                ##################LEFT TRIPOD #################
                ###################### DANCE BREAKKKKKKKKIEEEEEEEEEEEEEEE ###########################################
                if (t_c - t_s + (t_s)/2 +1.0) <= t < (t_c - t_s + (t_s)/2 +2.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                ###################### DANCE BREAKKKKKKKKIEEEEEEEEEEEEEEE ###########################################
                #### [0, phi_s/2] #### 
                elif (t_c - t_s + (t_s)/2 +2.0) <= t < (t_c - t_s  + 5* (t_s)/8 +2.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_c - t_s  + 5* (t_s)/8 +2.0) <= t < (t_c - t_s  + 3* (t_s)/4 +2.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/4)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_c - t_s  + 3* (t_s)/4 +2.0) <= t < (t_c - t_s + 7* (t_s)/8 +2.0) :
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (3* phi_s/8)
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_c - t_s + 7* (t_s)/8 +2.0) <= t < (t_c+2.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2)
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                #### [phi_s/2, pi] #####
                elif 0 <= t < (t_c - t_s)/8:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (phi_s/2 + (math.pi+phi_s/2)/2) /2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                elif ((t_c - t_s)/8) <= t < ((t_c - t_s)/4):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+phi_s/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                
                        
                elif ((t_c - t_s)/4 ) <= t < (3* (t_c - t_s)/8 ):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi + (math.pi+phi_s/2)/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif (3* (t_c - t_s)/8 ) <= t < (t_s/2 ):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = math.pi
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                        
                #### DANCE BREAK ####################################################################################
                elif (t_s/2 ) <= t < (t_s/2 ) + 0.5:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi)/2
                        self.cmd_vel[i] = 0.0
                        
                elif ( t_s/2 + 0.5) <= t < ( t_s/2 + 1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi)
                        self.cmd_vel[i] = 0.0
                #### DANCE BREAK ####################################################################################
                
                #### [-pi, -phi_s/2]
                elif ((t_c - t_s)/2 + 1.0) <= t < (5* (t_c - t_s)/8 + 1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi +(math.pi+(2*math.pi - phi_s/2))/2)/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)  
                        
                elif  ( 5* (t_c - t_s)/8 + 1.0) <= t < (3* (t_c - t_s)/4 + 1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (math.pi+(2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s) 
                

                elif (3* (t_c - t_s)/4 +1.0) <= t < (7* (t_c - t_s)/8 +1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = ((math.pi+(2*math.pi - phi_s/2))/2 + (2*math.pi - phi_s/2))/2
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                elif  (7* (t_c - t_s)/8 +1.0) <= t < ((t_c - t_s)+1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = (2*math.pi - phi_s/2)
                        self.cmd_vel[i] = (2* math.pi - phi_s)/(t_c - t_s)
                
                
                #### [-phi_s/2, 0]
                elif ((t_c - t_s)+1.0) <= t < ((t_c - t_s  + (t_s)/8)+1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3* phi_s/8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_c - t_s  + (t_s)/8 +1.0)  <= t < (t_c - t_s  + (t_s)/4 +1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s/4
                        self.cmd_vel[i] = (phi_s/ t_s)
                
                elif (t_c - t_s  + (t_s)/4 +1.0) <= t < (t_c - t_s + 3*(t_s)/8 +1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -phi_s /8
                        self.cmd_vel[i] = (phi_s/ t_s)
                        
                elif (t_c - t_s + 3*(t_s)/8 +1.0) <= t < (t_c - t_s + (t_s)/2 +1.0):
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = (phi_s/ t_s)
                
     
  
             
        self.simple_walker_enable = self.get_parameter('simple_walker_enable').get_parameter_value().bool_value
        self.state = self.get_parameter('state').get_parameter_value().integer_value

        if self.newdata:
            torque = Float64MultiArray()
            torque.data = self.compute_controls()
            if (self.simple_walker_enable):
                self.publisher.publish(torque)
            self.newdata = False
        
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
    
