import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

robot = LaunchConfiguration('robot')
robot_launch_arg = DeclareLaunchArgument('robot')

node_name = LaunchConfiguration('node_name')
node_name_launch_arg = DeclareLaunchArgument('node_name')

topic_remappings = [
    (['/input/image_raw'], ['/hercules_node/', robot, '/front_center_Scene/image']),
    (['/output/image/compressed'], ['/hercules_node/', robot, '/front_center_Scene/image/compressed']),
]

image_resize_compress_node = Node(
    package='roman_ros2',
    namespace='',
    executable='image_resize_compress_node.py',
    name=node_name,
    output='screen',
    emulate_tty=True,
    remappings=topic_remappings
)

def generate_launch_description():
    return LaunchDescription([
        robot_launch_arg,
        node_name_launch_arg,
        image_resize_compress_node
])

