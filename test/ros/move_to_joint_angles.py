#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import argparse
import time


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Move the robot to a set of joint angles.")
    parser.add_argument("joint_angles", type=float, nargs=6, help="A list of 6 joint angles")
    args = parser.parse_args()
    try:
        node = rclpy.create_node("move_to_joint_angles")
        # Jazzy defaults to scaled controller.
        action_topics = [
            "/joint_trajectory_controller/follow_joint_trajectory",
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        ]
        action_client = None
        for action_topic in action_topics:
            candidate = ActionClient(node, FollowJointTrajectory, action_topic)
            if candidate.wait_for_server(timeout_sec=5.0):
                action_client = candidate
                node.get_logger().info(f"Using action server: {action_topic}")
                break
        if action_client is None:
            node.get_logger().error("Action server not available")
            return

        joint_angles = args.joint_angles
        joint_angles = list(map(lambda x: math.pi * x / 180, joint_angles))
        print(joint_angles)

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Create a trajectory point with the desired joint angles
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 3

        trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        goal_tolerance = []
        for name in trajectory.joint_names:
            tolerance = JointTolerance()
            tolerance.name = name
            tolerance.position = 0.01
            tolerance.velocity = 0.0
            tolerance.acceleration = 0.0
            goal_tolerance.append(tolerance)


        goal.goal_tolerance = goal_tolerance

        future = action_client.send_goal_async(goal)
        time.sleep(5)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(str(future.result()))
        if future.result() is not None:
            node.get_logger().info("Robot reached the desired joint angles")
        else:
            node.get_logger().error("Failed to reach the desired joint angles")

    except Exception as e:
        if "node" in locals():
            node.get_logger().error(f"An error occurred: {str(e)}")
        else:
            print(f"An error occurred before node initialization: {str(e)}")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
