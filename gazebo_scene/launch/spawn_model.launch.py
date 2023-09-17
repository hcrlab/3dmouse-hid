from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            parameters=[{'x':0.0,'y':0 ,'z':0}],
            arguments=['-entity', 'my_robot', '-file moveit_resources/panda_description/urdf/panda.urdf'],



        )
    ])