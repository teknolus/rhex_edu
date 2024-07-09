#!/usr/bin/env python3
from math import pi
import socket
import threading
import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt64
from sensor_msgs.msg import JointState, Imu

def posdif(f, t):	#compute position 
	dif = (t-f) % (2*pi)
	return dif if dif<pi else dif - 2*pi

class LegController(Node):
	def __init__(self):
		super().__init__('leg_controller')
		self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
		self.create_subscription(UInt64, "/kbi", self.kbicb, 10)
		self.create_subscription(JointState, '/joint_states', self.jscb, 10)

		self.re = 2**4
		self.create_timer(1/self.re, self.ru)

		#leg indices
		"""
		l3 ^^ r3
		l2 || r2
		l1 || r1
		"""
		self.l1i=0
		self.l2i=2
		self.l3i=4
		self.r1i=1
		self.r2i=3
		self.r3i=5
		"""
		self.currTime = 0
		self.currPos = np.zeros(6)
		self.currVel = np.zeros(6)
		self.currTorq = np.zeros(6)
		self.currPose = np.zeros(4)
		self.globalPos = np.zeros(3)

		self.cmd_time = 0
		self.cmd_pos = np.zeros(6)
		self.cmd_kp = np.zeros(6)
		self.cmd_vel = np.zeros(6)
		self.cmd_kd = np.zeros(6)
		self.cmd_tau = np.zeros(6)
		self.cmd_enabled = False
		"""
		self.torque = Float64MultiArray()
		self.torque.data = [0.0]*6   #initial torque data.

		self.c = None	#command

		self.torop = [0]*6	#torque output

		self.pos = [0]*6	#updated every time self.jscb callback is called
		self.poso = [0]*6	#position old, to obtain velocities in the resolution of simulation

		self.posaim = [0]*6
		self.posaimk = 12
		self.walka = 2**-1
		self.walkn = 2
		self.t=0	#tick is increased for every step
		self.s=-1	#state

		self.q=0	#quit the controller if it is 1

		self.get_logger().info("\n\nThe node is ready!n\n")
		return
	def ru(s):
		s.t += 1
		if s.t % 1 == 0:	#speedy walk
			s.s += 1
			s.s %= (s.walkn) * 2	#there are as many states as 2 walkn's

		s.torop = [0]*6

		match s.s:
			case n if n < s.walkn:	#3 specific legs are running faster than other 3
				s.posaim[s.l1i] = -s.walka + 2 * s.walka * n / s.walkn
				s.posaim[s.r2i] = -s.walka + 2 * s.walka * n / s.walkn
				s.posaim[s.l3i] = -s.walka + 2 * s.walka * n / s.walkn
				s.posaim[s.r1i] = s.walka + 2 * (pi - s.walka) * n / s.walkn
				s.posaim[s.l2i] = s.walka + 2 * (pi - s.walka) * n / s.walkn
				s.posaim[s.r3i] = s.walka + 2 * (pi - s.walka) * n / s.walkn
			case m:	#alternate legs
				m -= s.walkn
				s.posaim[s.l1i] = s.walka + 2 * (pi - s.walka) * m / s.walkn
				s.posaim[s.r2i] = s.walka + 2 * (pi - s.walka) * m / s.walkn
				s.posaim[s.l3i] = s.walka + 2 * (pi - s.walka) * m / s.walkn
				s.posaim[s.r1i] = -s.walka + 2 * s.walka * m / s.walkn
				s.posaim[s.l2i] = -s.walka + 2 * s.walka * m / s.walkn
				s.posaim[s.r3i] = -s.walka + 2 * s.walka * m / s.walkn
 
		for lgi in range(6):	#calculate torque according to position aim and current pos
			s.torop[lgi] = posdif(s.pos[lgi], s.posaim[lgi]) * s.posaimk

		s.torque.data = [float(tor) for tor in s.torop]
		s.publisher.publish(s.torque)
		return

		if not s.gts():
			#s.gts().add(mlgspos(s, range(6), -0/2, 2, 2**-1))
			match s.s:
				case 0:
					s.gts().add(mlgdea(s, s.r1i, 2*pi))
					s.gts().add(mlgdea(s, s.r3i, 2*pi))
					s.gts().add(mlgdea(s, s.l2i, 2*pi))
					s.gts().add(mlgspos(s, {s.l1i, s.l3i, s.r2i}, pi))
					s.s += 1
				case 1:
					s.gts().add(mlgspos(s, {s.r1i, s.r3i, s.l2i}, pi))
					s.gts().add(mlgdea(s, s.l1i, 2*pi))
					s.gts().add(mlgdea(s, s.l3i, 2*pi))
					s.gts().add(mlgdea(s, s.r2i, 2*pi))
					s.s = 0
			"""
			"""
			"""
				case 2:
					s.gts().add(mlgdea(s, s.r1i, 2*pi, 6))
					s.gts().add(mlgdea(s, s.r3i, 2*pi, 6))
					s.gts().add(mlgdea(s, s.l2i, 2*pi, 6))
					s.gts().add(mlgpos(s, s.l1i, 0, 6))
					s.gts().add(mlgpos(s, s.l3i, 0, 6))
					s.gts().add(mlgpos(s, s.r2i, 0, 6))
					s.s = 0
			"""

		s.ts[not s._tsi].clear()
		for t in s.ts[s._tsi]:
			t.do()
			if not t.don:	s.ts[not s._tsi].add(t)
		s._tsi = not s._tsi

		s.torque.data = [float(tor) for tor in s.torop]
		s.publisher.publish(s.torque)
	
	def kbicb(s, m:UInt64):	#keyboard input callback from kbi.py
		match m.data:
			case 0:	s.ts[s._tsi] = set()
			case 10:	s.get_logger().info(f"s: {s.s}")
			case 3:	s.get_logger().info(f"{s.poso} -> {s.pos}")
			case 11:	s.q = 1
		"""
			#case 1:	s.ts[s._tsi].add(mlgdea(s, s.l1i, 2*pi))
			case 2:
				c=0
				for t in s.ts[s._tsi]:
					s.get_logger().info(f"{c}: {t.dea}")
					c+=1
		match m.data:
			case 0:	s.c = "sd"
			case 1:	s.c = "g1"
			case 2: s.c = "g2"
			case 3: s.c = "tr"
		s.s=0
		"""

	def jscb(self, m:JointState):	#joint state callback
		self.poso = self.pos
		self.pos = constrain_angle(np.array([*m.position]))
		return

		self.posdiso = self.posdis		
		self.currPos = constrain_angle(np.array([*m.position]))
		self.posdis = [posipd(pos) for pos in self.currPos]

		l1i=0
		l2i=2
		l3i=4
		r1i=1
		r2i=3
		r3i=5
		
		self.torop = [0]*6


			
		match self.c:
			case "sd":
				for i in range(6):
					self.mt(i, -1)
			case "g1":
				self.mt(l2i, 2)
				self.mt(r2i, 2)

				mt=0
				match self.s:
					case 0:
						mt += self.mt(l1i, -1)
						mt += self.mt(r1i, -1)
						mt += self.mt(l3i, -1)
						mt += self.mt(r3i, -1)
						if mt<2:
							self.s = 1
					case 1:
						mt += self.go(l1i, 8)
						mt += self.mt(r1i, -1)
						mt += self.mt(l3i, -1)
						mt += self.go(r3i, 8)
						if mt<1:
							self.s = 2
					case 2:
						mt += self.mt(l1i, -1)
						mt += self.mt(r1i, -1)
						mt += self.mt(l3i, -1)
						mt += self.mt(r3i, -1)
						if mt<2:
							self.s = 3
					case 3:
						mt += self.mt(l1i, -1)
						mt += self.go(r1i, 8)
						mt += self.go(l3i, 8)
						mt += self.mt(r3i, -1)
						if mt<1:
							self.s = 0
			case "g2":    
				self.mt(l2i, 2)
				self.mt(r2i, 2)

				mt = 0
				match self.s:
					case 0:
						mt += self.mt(l1i, 0)
						mt += self.mt(r1i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r3i, 0)
						if mt<2:
							self.s = 1
							print(f"0->1")
					case 1:
						mt += self.mt(l1i, -2)
						mt += self.mt(r1i, -2)
						mt += self.mt(l3i, -2)
						mt += self.mt(r3i, -2)
						if mt<2:
							self.s = 0
							print(f"1->0")
			case "tr":
				self.mt(l2i, 2)
				self.mt(r2i, 2)

				mt=0
				match self.s:
					case 0:
						mt += self.mt(l1i, -1)
						mt += self.mt(r1i, -1)
						mt += self.mt(l3i, -1)
						mt += self.mt(r3i, -1)
						if mt<2:
							self.s = 1
					case 1:
						mt += self.go(l1i, 16)
						mt += self.mt(r1i, -1)
						mt += self.mt(l3i, -1)
						mt += self.go(r3i, 8)
						if mt<1:
							self.s = 2
					case 2:
						mt += self.mt(l1i, -1)
						mt += self.mt(r1i, -1)
						mt += self.mt(l3i, -1)
						mt += self.mt(r3i, -1)
						if mt<2:
							self.s = 3
					case 3:
						mt += self.mt(l1i, -1)
						mt += self.go(r1i, 8)
						mt += self.go(l3i, 16)
						mt += self.mt(r3i, -1)
						if mt<1:
							self.s = 0
			case ".":
				self.mt(l2i, 2)
				self.mt(r2i, 2)

				mt=0
				match self.s:
					case 0:
						mt += self.mt(l1i, 0)
						mt += self.mt(r1i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r3i, 0)
						if mt<2:
							self.s = 1
							"""
							print(f"torop: {self.torop}")
							print(f"curvel: {self.currVel}")
							print(f"state: {self.s}")
							"""
					case 1:
						mt += self.mt(l1i, -2)
						mt += self.mt(l3i, -2)
						mt += self.mt(r1i, -2)
						mt += self.mt(r3i, -2)
						if mt<2:
							self.s = 2
					case 2:
						mt += self.mt(l1i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r1i, -2)
						mt += self.mt(r3i, -2)
						if mt<2:
							self.s = 0
		
		self.torque.data = [float(tor) for tor in self.torop]
		self.publisher.publish(self.torque)

def main(args=None):
	rclpy.init(args=args)
	legController = LegController()
	try:
		while 1:	#main loop for taking callbacks (timer callback counts)
			rclpy.spin_once(legController)
			if legController.q == 1:	break	#for not having to ctrl*c and have errors
		rclpy.shutdown()	#gives errors most of the time.
	except Exception as e:
		print(e)
	finally:
		rclpy.shutdown()	#gives errors most of the time.
	

if __name__ == '__main__':
	main()
