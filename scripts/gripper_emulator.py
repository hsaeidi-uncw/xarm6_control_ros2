#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# The standard gripper command message types
from control_msgs.action import GripperCommand

class XarmGripperMockServer(Node):
    def __init__(self):
        super().__init__('xarm_gripper_mock_server')
        
        # Create the action server (the real robot has it but NOT the simulator, so we have to create a mock version here for the projects)
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/xarm_gripper/gripper_action',
            self.execute_callback
        )
        
        self.get_logger().info('xArm Gripper Mock Action Server has been started.')
        self.get_logger().info('Ready to receive goals on /xarm_gripper/gripper_action')

    async def execute_callback(self, goal_handle):
        """This function runs when a goal is received."""
        target_position = goal_handle.request.command.position
        max_effort = goal_handle.request.command.max_effort
        
        self.get_logger().info(f'Executing goal: Target Position={target_position}, Max Effort={max_effort}')

        # Simulate "movement" time
        # In a real robot, this would wait for the joints to reach the state
        time.sleep(1.0) 

        # Mark the goal as successful
        goal_handle.succeed()

        # Prepare the result like the CLI outputs
        result = GripperCommand.Result()
        # send the same number as feedback since the gripper does not work in the simulator
        result.position = target_position 
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True # Set to true since we succeeded

        self.get_logger().info(f'Goal finished with status: SUCCEEDED. Final Pos: {result.position}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    
    node = XarmGripperMockServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
