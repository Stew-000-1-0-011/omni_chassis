"""chassis_node を起動する。bridge:=true で robomas_bridge も一緒に立てる。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params')
    bridge = LaunchConfiguration('bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('omni_chassis'), 'config', 'chassis_node.yaml']),
            description='chassis_node のパラメータファイル',
        ),
        DeclareLaunchArgument(
            'bridge', default_value='false',
            description='robomas_bridge (robomas_plugins) も一緒に起動する',
        ),
        Node(
            package='omni_chassis',
            executable='chassis_node',
            name='chassis_node',
            parameters=[params],
            output='screen',
        ),
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='robomas_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='robomas_plugins',
                    plugin='robomas_bridge::RobomasBridge',
                    name='robomas_bridge',
                ),
            ],
            output='screen',
            condition=IfCondition(bridge),
        ),
    ])
