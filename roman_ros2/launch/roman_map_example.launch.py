import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = LaunchConfiguration('robot')
node_name = LaunchConfiguration('node_name')
config_path = LaunchConfiguration('config_path')

robot_launch_arg = DeclareLaunchArgument('robot')
node_name_launch_arg = DeclareLaunchArgument('node_name')
config_path_launch_arg = DeclareLaunchArgument('config_path',
    default_value=os.path.join(
    get_package_share_directory('roman_ros2'), 'cfg', 'default_mapper.yaml'))


topic_remappings = [
    ('color/camera_info', ['hercules_node/', robot, '/front_center_Scene/camera_info']),
    ('color/image_raw', ['hercules_node/', robot, '/front_center_Scene/image']),
    ('roman/observations', ['roman/', robot, '/observations']),
    ('roman/roman_map/status', ['roman/', robot, '/mapper/status']),
    ('roman/annotated_img', ['roman/', robot, '/annotated_img']),
    ('roman/pulse', ['roman/', robot, '/pulse']),
    ('roman/object_points', ['roman/', robot, '/object_points']),
    ('roman/segment_updates', ['roman/', robot, '/segment_updates']),
]

tf_remappings = [
    (['/', robot, '/tf'], '/tf'),
    (['/', robot, '/tf_static'], '/tf_static'),
]

frame_params = {
    'map_frame_id': 'world',
    'odom_base_frame_id': [robot, '/odom_local'],
    'use_sim_time': True,
    'wait_for_tf_time': 10.0,
}

config_path_param = {'config_path': config_path}

def generate_launch_description():
    return LaunchDescription([
        robot_launch_arg,
        node_name_launch_arg,
        config_path_launch_arg,
        Node(
            package='roman_ros2',
            namespace='',
            executable='roman_map_node.py',
            name=node_name,
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(get_package_share_directory('roman_ros2'), 'cfg', 'default_roman_map.yaml'), frame_params, config_path_param],
            remappings=topic_remappings + tf_remappings
        )
])