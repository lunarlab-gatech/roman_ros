import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

roman_align_node = Node(
    package='roman_ros2',
    namespace='',
    executable='test_depthperspective.py',
    name=['test_depth'],
    output='screen',
    emulate_tty=True,
)

def generate_launch_description():
    return LaunchDescription([
        roman_align_node
])

