"""chassis_node を起動する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            description='chassis_node のパラメータファイル (必須。chassis_node は既定値を持たない)',
        ),
        Node(
            package='omni_chassis',
            executable='chassis_node',
            name='chassis_node',
            parameters=[params],
            output='screen',
        ),
    ])
