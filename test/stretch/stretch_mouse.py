
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int64
import stretch_body.robot
import time


class TwistSubscriber(Node):
    def __init__(self, robot):
        super().__init__('twist_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/stretch_teleop/mouse',
            self.listener_callback,
            1)
        self.button_subscription = self.create_subscription(
            Int64,
            '/stretch_teleop/buttons',
            self.gripper,
            1)
        self.robot = robot
        self.robot.head.pose('tool')
        self.gripper_is_closed = False
        self.last_command_time = time.time()

    def listener_callback(self, msg):
        self.linear_movements(msg)
        self.angular_movements(msg)
        self.mode=None

        

    def linear_movements(self, msg):
        linear_x=round(msg.twist.linear.x, 1)
        linear_y=round(msg.twist.linear.y, 1)
        linear_z=round(msg.twist.linear.z, 1)
        commanded = False
        if(linear_x==0 or linear_x>0 or linear_x<0):
            self.robot.arm.move_by(linear_x*-0.2)
            commanded = True
        if(linear_y==0 or linear_y>0 or linear_y<0):
            self.robot.base.translate_by(linear_y*-0.2)
            commanded = True
        if(linear_z==0 or linear_z>0 or linear_z<0):
            self.robot.lift.move_by(linear_z*0.2)
            commanded = True
        if commanded:
            self.robot.push_command()
    
        
       
    def angular_movements(self, msg):
        angular_x=round(msg.twist.angular.x, 1)
        angular_y=round(msg.twist.angular.y, 1)
        angular_z=round(msg.twist.angular.z, 1)
        commanded = False
        if(angular_x==0 or angular_x>0 or angular_x<0):
            self.robot.end_of_arm.move_by('wrist_roll', angular_x*-.4)
            commanded = True
        if(angular_y==0 or angular_y>0 or angular_y<0):
            self.robot.end_of_arm.move_by('wrist_pitch', angular_y*.4)
            commanded = True
        if(angular_z==0 or angular_z>0 or angular_z<0):
            self.robot.end_of_arm.move_by('wrist_yaw', angular_z*.4)
            commanded = True
        if commanded:
            self.robot.push_command()

    def gripper(self, msg):
        if msg.data == 0:
            # We're mapping any button to toggle the gripper
            return
        time_since_last_command = time.time() - self.last_command_time
        if time_since_last_command < 0.5:
            return
        if self.gripper_is_closed:
            # Close the gripper
            self.robot.end_of_arm.move_to('stretch_gripper',-60)
            self.gripper_is_closed = False
        elif not self.gripper_is_closed:
            # Open the gripper
            self.robot.end_of_arm.move_to('stretch_gripper',200)
            self.gripper_is_closed = True


def main(args=None):
    rclpy.init(args=args)
    robot=stretch_body.robot.Robot()
    robot.startup()
    twist_subscriber = TwistSubscriber(robot)
    rclpy.spin(twist_subscriber)
    # Someone ctrl c-ed us
    robot.stop()
    twist_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
