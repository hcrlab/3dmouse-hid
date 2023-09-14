# import rclpy
# from rclpy.duration import Duration
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# # import pdb; pdb.set_trace()

# class GripperControlNode(Node):
#     def __init__(self):
#         super().__init__('gripper_control_node')
#         self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
#         print("varad")
        

#     def move_grip(self):
#         if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error('Action server not available!')
#             return
#         joint_name = ["panda_finger_joint1", "panda_finger_joint2"]
#         goal_msg = FollowJointTrajectory.Goal()
#         duration1 = rclpy.duration.Duration(seconds=0.0)
#         duration2 = rclpy.duration.Duration(seconds=2.0)
        
        
#         point1 = JointTrajectoryPoint()
#         point2 = JointTrajectoryPoint()
#         point1.time_from_start = duration1.to_msg()
#         point2.time_from_start = duration2.to_msg()
      

#         point1.positions = [0.04, 0.04]
#         point2.positions = [0.0, 0.0]
#         goal_msg.trajectory.joint_names = joint_name
#         goal_msg.trajectory.points = [point1, point2]
#         print("okakaka111111")
#         goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
#         self.get_logger().info('Sending goal to the action server...')
    
#         print("okakaka")
#         self.trajectory_client.send_goal(goal_msg)
#         print("ok")
#         self.trajectory_client.wait_for_result()
#         print("not oaky")

# def main(args=None):
#     rclpy.init(args=args)
#     node = GripperControlNode()
#     node.move_grip()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')

    def open_gripper(self):
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04  # Set the desired gripper position (open)
        goal_msg.command.max_effort = 2.0  # Set the max effort (adjust as needed)

        self.get_logger().info('Sending open gripper command...')
        self.gripper_action_client.send_goal(goal_msg)

        self.get_logger().info('Waiting for open gripper action to complete...')
        result = self.gripper_action_client.wait_for_result()
        if not result:
            self.get_logger().warn('Open gripper action did not complete successfully.')
        else:
            self.get_logger().info('Open gripper action completed successfully.')

    def close_gripper(self):
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.00  # Set the desired gripper position (close)
        goal_msg.command.max_effort = 1.0  # Set the max effort (adjust as needed)

        self.get_logger().info('Sending close gripper command...')
        self.gripper_action_client.send_goal(goal_msg)

        self.get_logger().info('Waiting for close gripper action to complete...')
        result = self.gripper_action_client.wait_for_result()
        if not result:
            self.get_logger().warn('Close gripper action did not complete successfully.')
        else:
            self.get_logger().info('Close gripper action completed successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    
    # Open the gripper
    node.open_gripper()

    # Perform other actions or wait as needed

    # Close the gripper
    node.close_gripper()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

