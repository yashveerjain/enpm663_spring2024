#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
 
def main(arg=None):
    # 1. Initialize ROS communications for a given context
    rclpy.init(args=arg)
    # 2. Instantiate a Node
    node = Node("minimal_node_py")
    node.get_logger().info(f"Hello from {node.get_name()}")
    # 3. Shutdown a previously initialized context
    rclpy.shutdown()

if __name__ == "__main__":
    main()