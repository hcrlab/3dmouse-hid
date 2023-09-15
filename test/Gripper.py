import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from std_msgs.msg import Empty

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        open_gripper_subscription=self.create_subscription(Empty,'/open_grip', self.open_gripper,10)
        close_gripper_subscription=self.create_subscription(Empty,'/close_grip', self.close_gripper,10)

    def open_gripper(self,msg):
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04  # Set the desired gripper position (open)
        goal_msg.command.max_effort = 2.0  # Set the max effort (adjust as needed)

        self.get_logger().info('Sending open gripper command...')
        goal_handle_future = self.gripper_action_client.send_goal_async(goal_msg)
        goal_handle_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Waiting for open gripper action to complete...')
        

    def close_gripper(self,msg):
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.00  # Set the desired gripper position (close)
        goal_msg.command.max_effort = 1.0  # Set the max effort (adjust as needed)

        self.get_logger().info('Sending close gripper command...')
        goal_handle_future = self.gripper_action_client.send_goal_async(goal_msg)
        goal_handle_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Waiting for close gripper action to complete...')
        
       

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):
        result = future.result().result
        if result is not None:
            self.get_logger().info('Goal succeeded! Result: {result}')
        else:
            self.get_logger().info('Goal failed with status code: {future.result().status}')


            
def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

