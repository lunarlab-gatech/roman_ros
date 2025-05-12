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
    get_package_share_directory('roman_ros2'), 'cfg', 'default_fastsam.yaml'))


topic_remappings = [
    ('color/camera_info', ['hercules_node/', robot, '/front_center_Scene/camera_info']),
    ('color/image_raw', ['hercules_node/', robot, '/front_center_Scene/image']),
    ('depth/camera_info', ['hercules_node/', robot, '/front_center_DepthPerspective/camera_info']), # assumes aligned color/depth images
    ('depth/image_raw', ['hercules_node/', robot, '/front_center_DepthPerspective/image']),
    ('roman/observations', ['roman/', robot, '/observations']),
    ('roman/fastsam/status', ['roman/', robot, '/fastsam/status']),
#,
]

tf_remappings = [
    (['/', robot, '/tf'], '/tf'),
    (['/', robot, '/tf_static'], '/tf_static'),
]

frame_params = {
    'map_frame_id': 'world',
    'odom_base_frame_id': [robot, '/odom_local'],
    'cam_frame_id': [robot, '/front_center_optical'],
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
            executable='fastsam_node.py',
            name=node_name,
            output='screen',
            emulate_tty=True,
            parameters=[config_path_param, frame_params],
            remappings=topic_remappings + tf_remappings
        )
])