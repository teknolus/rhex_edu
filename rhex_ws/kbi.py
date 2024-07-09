#!/usr/bin/env python3
from readchar import readchar as rc
from asyncio import run as r
import asyncio

from math import pi

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
from std_msgs.msg import Float64MultiArray, UInt64
from sensor_msgs.msg import JointState, Imu, PointCloud2
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from gazebo_msgs.srv import GetModelState, GetEntityState
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from random import randrange as rr
from teleop_twist_keyboard import getKey as gk

class kbi(Node):
	def __init__(self):
		super().__init__('kbi')
		self.publisher = self.create_publisher(UInt64, "/kbi", 10)

def main(args=None):
	rclpy.init(args=args)
	
	kbi0 = kbi()

	print("Hello. With this terminal, you can send commands to the /kbi topic via key-presses. Try them out.")
	
	msg=UInt64()
	while 1:
		match rc():
			case "s":	msg.data = 10
			case ".":	msg.data = 11
			case "q":	break
			case n:
				msg.data = int(n)
		kbi0.publisher.publish(msg)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
