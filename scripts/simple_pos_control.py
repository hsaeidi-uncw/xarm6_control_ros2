#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class SimplePosControl(Node):
    def __init__(self):
        super().__init__('simple_pos_control')

        # Define the target joint names we care about (the driver adds a weird gripper joint add the beginning!)
        self.target_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Subscribers and Publishers
        # Read the joint values
        self.create_subscription(JointState, '/joint_states', self.pose_callback, 10)
        
        # Publisher for the Joint Trajectory Controller
        self.pos_pub = self.create_publisher(
            JointTrajectory, 
            '/xarm6_traj_controller/joint_trajectory', 
            10
        )

        # Timer for the control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Prepare the command message template
        self.pos_cmd = JointTrajectory()
        self.pos_cmd.joint_names = self.target_joint_names
        
        self.get_logger().info('ROS 2 Simple Position Control Node Started')

    def pose_callback(self, msg):
        """
        Finds the correct joint index by name to avoid 
        issues with extra joints (like grippers) in the array.
        """
        found_data = []
        for name in self.target_joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                pos = msg.position[idx]
                found_data.append(f"{name}: {pos:.2f}")
            
        # Log the filtered joint data
        if found_data:
            self.get_logger().info(f"Filtered States -> {', '.join(found_data)}")

    def control_loop(self):
        # Create a new trajectory point
        point = JointTrajectoryPoint()
        
        # Initialize 6 joints to 0.0
        point.positions = [0.0] * 6
        
        # Set target positions (matches your original logic)
        point.positions[1] = -math.pi / 4  # joint2
        point.positions[2] = -math.pi / 2  # joint3
        
        # ROS 2 duration uses seconds and nanoseconds
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        # Update message and publish     
        self.pos_cmd.points = [point]
        
        self.pos_pub.publish(self.pos_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePosControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
