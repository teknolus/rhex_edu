#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CommandPublisherNode(Node):

    def __init__(self):
        super().__init__('command_publisher_node')
        
        # Publishers for each command
        self.pub_cmd_tau = self.create_publisher(Float64MultiArray, 'cmd_tau', 10)
        self.pub_cmd_vel = self.create_publisher(Float64MultiArray, 'cmd_vel', 10)
        self.pub_cmd_pos = self.create_publisher(Float64MultiArray, 'cmd_pos', 10)
        self.pub_cmd_kd = self.create_publisher(Float64MultiArray, 'cmd_kd', 10)
        self.pub_cmd_kp = self.create_publisher(Float64MultiArray, 'cmd_kp', 10)

        # Timer for publishing commands (adjust the interval as needed)
        self.timer_ = self.create_timer(0.1, self.publish_commands)

    def publish_commands(self):
        # Example data for each command
        cmd_tau = Float64MultiArray(data=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        cmd_vel = Float64MultiArray(data=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        cmd_pos = Float64MultiArray(data=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        cmd_kd = Float64MultiArray(data=[0.01, 0.02, 0.03, 0.04, 0.05, 0.06])
        cmd_kp = Float64MultiArray(data=[100.0, 200.0, 300.0, 400.0, 500.0, 600.0])
        
        # Publishing each command
        self.pub_cmd_tau.publish(cmd_tau)
        self.pub_cmd_vel.publish(cmd_vel)
        self.pub_cmd_pos.publish(cmd_pos)
        self.pub_cmd_kd.publish(cmd_kd)
        self.pub_cmd_kp.publish(cmd_kp)

        # Logging the published values
        self.get_logger().info('Publishing cmd_tau: %s' % str(cmd_tau.data))
        self.get_logger().info('Publishing cmd_vel: %s' % str(cmd_vel.data))
        self.get_logger().info('Publishing cmd_pos: %s' % str(cmd_pos.data))
        self.get_logger().info('Publishing cmd_kd: %s' % str(cmd_kd.data))
        self.get_logger().info('Publishing cmd_kp: %s' % str(cmd_kp.data))

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
