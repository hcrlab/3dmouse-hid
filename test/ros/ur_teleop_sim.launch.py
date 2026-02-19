from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch Python API: https://docs.ros.org/en/ros2_packages/rolling/api/launch/launch.actions.html
def generate_launch_description():
    start_rosbridge = LaunchConfiguration("start_rosbridge")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    ur_gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(["ur_sim_moveit.launch.py"]),
        launch_arguments={
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
            "world_file": world_file,
        }.items(),
    )
    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        condition=IfCondition(start_rosbridge),
    )
    enable_servo = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service",
            "call",
            "/servo_node/start_servo",
            "std_srvs/srv/Trigger",
        ],
        shell=True,
    )
    spawn_cameras = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "ros_gz_sim",
            "create",
            "-file",
            "camera_rig_gz.sdf",
            "-name",
            "camera_rig",
            "-allow_renaming",
            "true",
        ],
        output="screen"
    )
    camera_image_bridge = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "ros_gz_image",
            "image_bridge",
            "/camera0/image_raw",
            "/camera1/image_raw",
        ],
        output="screen",
    )
    move_to_home_pose = ExecuteProcess(
        cmd=[
            "./move_to_joint_angles.py",
            "0.0 -108 90 -86 -88 -7",
        ],
        shell=True,
    )
    load_controller = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "control",
            "load_controller",
            "--set-state",
            "configured",
            "forward_position_controller",
        ],
        output="screen",
    )
    swap_controllers = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "control",
            "switch_controllers",
            "--deactivate",
            "joint_trajectory_controller",
            "--activate",
            "forward_position_controller",
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rosbridge",
            default_value="true",
            description="Start rosbridge_websocket in this launch.",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="false",
            description="Start Gazebo with the GUI enabled.",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="camera_world.sdf",
            description="SDF world file with sensors enabled.",
        ),
        ur_gazebo_sim,
        rosbridge_websocket,
        camera_image_bridge,
        TimerAction(period=10.0, actions=[move_to_home_pose, spawn_cameras]),
        RegisterEventHandler(OnProcessExit(target_action=move_to_home_pose, on_exit=[load_controller])),
        RegisterEventHandler(OnProcessExit(target_action=load_controller, on_exit=[swap_controllers])),
        RegisterEventHandler(OnProcessExit(target_action=swap_controllers, on_exit=[enable_servo])),
    ])
