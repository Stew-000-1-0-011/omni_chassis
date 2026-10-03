"""chassis_node を起動する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('omni_chassis'), 'config', 'chassis_node.yaml']),
            description='chassis_node のパラメータファイル',
        ),
        Node(
            package='omni_chassis',
            executable='chassis_node',
            name='chassis_node',
            parameters=[params],
            output='screen',
        ),
    ])
