import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = LaunchConfiguration('robot')
camera = LaunchConfiguration('camera')

robot_launch_arg = DeclareLaunchArgument('robot')
camera_launch_arg = DeclareLaunchArgument('camera')


topic_remappings = [
    ('color/camera_info', [camera, '/color/camera_info']),
    ('color/image_raw', [camera, '/color/image_raw']),
    ('depth/camera_info', [camera, '/color/camera_info']), # assumes aligned color/depth images
    ('depth/image_raw', [camera, '/aligned_depth_to_color/image_raw']),
]

tf_remappings = [
    (['/', robot, '/tf'], '/tf'),
    (['/', robot, '/tf_static'], '/tf_static'),
]

frame_params = {
    'cam_frame_id': [robot, '/', camera, '_color_optical_frame'],
    'map_frame_id': [robot, '/odom'],
    'odom_base_frame_id': [robot, '/base'],
}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roman_ros2',
            namespace=robot,
            executable='fastsam_node.py',
            name='fastsam_node',
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(get_package_share_directory('roman_ros2'), 'cfg', 'default_fastsam.yaml'), frame_params],
            remappings=topic_remappings + tf_remappings
        )
])