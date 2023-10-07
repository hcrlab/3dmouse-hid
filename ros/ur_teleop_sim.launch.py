from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

# ros2 launch Python API: https://docs.ros.org/en/ros2_packages/rolling/api/launch/launch.actions.html
def generate_launch_description():
    ur_gazebo_sim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_simulation_gazebo'), 'launch'),
         '/ur_sim_moveit.launch.py'])
      )
    rosbridge_websocket = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        )
    enable_servo = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'service',
            'call',
            '/servo_node/start_servo',
            'std_srvs/srv/Trigger',
        ],
        shell=True
    )
    spawn_cameras = ExecuteProcess(
        cmd=[
            './spawner.py',
            'camera_rig.urdf'
        ],
        shell=True
    )
    move_to_home_pose = ExecuteProcess(
        cmd=[
            './move_to_joint_angles.py',
            '0.0 -108 90 -86 -88 -7'
        ],
        shell=True
    )
    load_controller = ExecuteProcess(
    cmd= [FindExecutable(name='ros2'), "control" ,"load_controller", "--set-state", "configured", "forward_position_controller"], output="screen"
    )
    swap_controllers = ExecuteProcess(
    cmd = [FindExecutable(name='ros2'), "control", "switch_controllers", "--deactivate", "joint_trajectory_controller", "--activate", "forward_position_controller"], output="screen")
    return LaunchDescription([
        ur_gazebo_sim,
        rosbridge_websocket,
        TimerAction(period=10.0, actions=[move_to_home_pose, spawn_cameras]),
        RegisterEventHandler(OnProcessExit(target_action=move_to_home_pose, on_exit=[load_controller])),
        RegisterEventHandler(OnProcessExit(target_action=load_controller, on_exit=[swap_controllers])),
        RegisterEventHandler(OnProcessExit(target_action=swap_controllers, on_exit=[enable_servo]))
    ])