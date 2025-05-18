import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'config.rviz')

    return LaunchDescription([
        Node(
            package="robot_patrol",
            executable="direction_service_node",
            output="screen"
        ),
        Node(
            package="robot_patrol",
            executable="patrol_with_service_node",
            output="screen"
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')
    ])