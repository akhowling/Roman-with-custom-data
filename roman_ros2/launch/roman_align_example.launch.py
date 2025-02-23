import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot1 = LaunchConfiguration('robot1')
robot2 = LaunchConfiguration('robot2')
robot1_launch_arg = DeclareLaunchArgument(
    'robot1',
    default_value=None
)
robot2_launch_arg = DeclareLaunchArgument(
    'robot2',
    default_value=None
)

roman_align_node_params = {
    "robot1": robot1,
    "robot2": robot2,
}

roman_align_node = Node(
    package='roman_ros2',
    namespace='',
    executable='roman_align_node.py',
    name=[robot1, '_', robot2, '_roman_align_node'],
    output='screen',
    emulate_tty=True,
    parameters=[roman_align_node_params],
)

def generate_launch_description():
    return LaunchDescription([
        robot1_launch_arg,
        robot2_launch_arg,
        roman_align_node
])

