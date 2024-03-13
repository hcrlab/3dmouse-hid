#!/usr/bin/env python3
import os
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import stretch_body.robot
import time

import numpy as np
import ikpy.chain
import stretch_body.robot

# disable numpy scientific notation
np.set_printoptions(suppress=True)

# pip install urchin graphviz ikpy


class TwistController(Node):
    def __init__(self, robot):
        super().__init__('twist_controller')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/threedmouse/twist',
            self.twist_handler,
            1)
        self.button_subscription = self.create_subscription(
            String,
            '/threedmouse/button_event',
            self.button_event,
            1)
        self.robot = robot
        self.robot.head.pose('tool')
        self.last_command_time = time.time()

    @property
    def gripper_is_closed(self):
        return self.robot.end_of_arm.status['wrist_pitch']['pos'] > 100

    def twist_handler(self, msg):
        linear_x = msg.twist.linear.x
        linear_y = msg.twist.linear.y
        linear_z = msg.twist.linear.z

        self.robot.arm.move_by(linear_x*-0.2)
        self.robot.base.translate_by(linear_y*-0.2)
        self.robot.lift.move_by(linear_z*0.2)

        angular_x= msg.twist.angular.x
        angular_y= msg.twist.angular.y
        angular_z= msg.twist.angular.z

        self.robot.end_of_arm.move_by('wrist_roll', angular_x*-.4)
        self.robot.end_of_arm.move_by('wrist_pitch', angular_y*.4)
        self.robot.end_of_arm.move_by('wrist_yaw', angular_z*.4)

        self.robot.push_command()

    def button_event(self, msg):
        button_name, button_event = msg.data.split(',')

        # We need the following commands to act using the latest gripper joint state
        self.robot.end_of_arm.pull_status()
        if button_name in {'CTRL', 'LEFT', 'RIGHT', 'MENU'} and button_event == 'up':
            # Stop the gripper movement
            #self.robot.end_of_arm.move_by('stretch_gripper',0)
            self.robot.end_of_arm.stop_trajectory()
            return

        if button_name == 'CTRL' or button_name == 'LEFT':
            self.robot.end_of_arm.move_to('stretch_gripper',-60)
        elif button_name == 'MENU' or button_name == 'RIGHT':
            self.robot.end_of_arm.move_to('stretch_gripper',200)
        elif button_name == 'FIT' and button_event == 'down':
            self.robot.end_of_arm.move_to('wrist_roll',0)
            self.robot.end_of_arm.move_to('wrist_pitch',0)
            self.robot.end_of_arm.move_to('wrist_yaw',0)
            self.robot.push_command()


def main(args=None):
    rclpy.init(args=args)
    robot=stretch_body.robot.Robot()
    robot.startup()
    twist_controller = TwistController(robot)
    rclpy.spin(twist_controller)
    # Someone ctrl c-ed us
    robot.stop()
    twist_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
