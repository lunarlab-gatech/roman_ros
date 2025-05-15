import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Declare various command line arguments
robot = LaunchConfiguration('robot')
robot_launch_arg = DeclareLaunchArgument('robot')

node_name = LaunchConfiguration('node_name')
node_name_launch_arg = DeclareLaunchArgument('node_name')

expected_fps = LaunchConfiguration('expected_fps')
expected_fps_launch_arg = DeclareLaunchArgument('expected_fps', default_value='30.0')

bag_play_rate = LaunchConfiguration('bag_play_rate')
bag_play_rate_launch_arg = DeclareLaunchArgument('bag_play_rate', default_value='1.0')

# Declere the topic remappings
topic_remappings = [
    (['/input/image_raw'], ['/roman/', robot, '/annotated_img']),
    #(['/input/image_raw'], ['/hercules_node/', robot, '/front_center_Scene/image']),
]

# Declare ROS parameters to pass to the node
params = {
    'robot': robot,
    'expected_fps': expected_fps,
    'use_sim_time': True,
    'bag_play_rate': bag_play_rate,
}

# Define the node and launch description
image_resize_compress_node = Node(
    package='roman_ros2',
    namespace='',
    executable='low_latency_image_viewer.py',
    name=node_name,
    output='screen',
    emulate_tty=True,
    parameters=[params],
    remappings=topic_remappings
)

def generate_launch_description():
    return LaunchDescription([
        robot_launch_arg,
        node_name_launch_arg,
        expected_fps_launch_arg,
        bag_play_rate_launch_arg,
        image_resize_compress_node
])
