import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = LaunchConfiguration('robot')
camera = LaunchConfiguration('camera')
config_path = LaunchConfiguration('config_path')

robot_launch_arg = DeclareLaunchArgument('robot')
camera_launch_arg = DeclareLaunchArgument('camera')
config_path_launch_arg = DeclareLaunchArgument('config_path',
    default_value=os.path.join(
    get_package_share_directory('roman_ros2'), 'cfg', 'default_mapper.yaml'))


topic_remappings = [
    ('color/camera_info', [camera, '/color/camera_info']),
    ('color/image_raw', [camera, '/color/image_raw']),
]

tf_remappings = [
    (['/', robot, '/tf'], '/tf'),
    (['/', robot, '/tf_static'], '/tf_static'),
]

frame_params = {
    'map_frame_id': [robot, '/odom'],
    'odom_base_frame_id': [robot, '/base'],
}

config_path_param = {'config_path': config_path}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roman_ros2',
            namespace=robot,
            executable='roman_map_node.py',
            name='roman_map_node',
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(get_package_share_directory('roman_ros2'), 'cfg', 'default_roman_map.yaml'), frame_params, config_path_param],
            remappings=topic_remappings + tf_remappings
        )
])