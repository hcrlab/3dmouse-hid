import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    ros_distro = os.environ.get("ROS_DISTRO", "").strip().lower()
    selected_launch = (
        "ur_sim_moveit_jazzy.launch.py"
        if ros_distro == "jazzy"
        else "ur_sim_moveit_humble.launch.py"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([selected_launch]),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "world_file": LaunchConfiguration("world_file"),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("world_file", default_value="camera_world.sdf"),
            OpaqueFunction(function=launch_setup),
        ]
    )
