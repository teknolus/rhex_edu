#!/usr/bin/env python3 

# TO BE ADDED 
import rclpy 
from rclpy.node import Node 

class SimpleWalker(Node):
    def __init__(self):
        super().__init__("simple_walker")
        self.get_logger().info("simple_walker initialized")
        
        
def main (args = None):
    rclpy.init(args = args)
    node = SimpleWalker()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()