#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# Import the built interface/message
from xarm6_control_ros2.msg import Plan

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        
        # Define the publisher
        self.plan_pub = self.create_publisher(Plan, '/plan', 10)
        
        # Create a timer to publish every 0.1 seconds (10Hz), double-check if I ended up changing any other loop rates!
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Define the plan once
        self.plan = Plan()
        
        # Point 1: Close to initial position
        p1 = Twist()
        p1.linear.x = 0.2
        p1.linear.y = 0.0
        p1.linear.z = 0.11
        p1.angular.x = -3.1415
        p1.angular.y = 0.0011
        p1.angular.z = 0.0002
        self.plan.points.append(p1)
        
        # Point 2: Away from initial position
        p2 = Twist()
        p2.linear.x = 0.3
        p2.linear.y = 0.0
        p2.linear.z = 0.2
        p2.angular.x = -3.1415
        p2.angular.y = 0.0011
        p2.angular.z = 0.0002
        self.plan.points.append(p2)
        
        self.get_logger().info('Simple Planner Node started, sending cyclic plan...')

    def timer_callback(self):
        # Publish the plan repeatedly
        self.plan_pub.publish(self.plan)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
