import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

robot1 = LaunchConfiguration('robot1')
robot2 = LaunchConfiguration('robot2')
cfg = LaunchConfiguration('cfg')

robot1_launch_arg = DeclareLaunchArgument(
    'robot1',
    default_value=None
)
robot2_launch_arg = DeclareLaunchArgument(
    'robot2',
    default_value=None
)
cfg_arg = DeclareLaunchArgument(
    'cfg',
    default_value=os.path.join(get_package_share_directory('roman_ros2'), 'cfg', 'default_roman_align.yaml')
)

roman_align_node_params = {
    "robot1": robot1,
    "robot2": robot2,
}

parameters = [roman_align_node_params]
cfg_set = IfCondition(cfg)
if cfg_set:
    parameters.append(cfg)

roman_align_node = Node(
    package='roman_ros2',
    namespace='',
    executable='roman_align_node.py',
    name=[robot1, '_', robot2, '_roman_align_node'],
    output='screen',
    emulate_tty=True,
    parameters=parameters,
)

def generate_launch_description():
    return LaunchDescription([
        robot1_launch_arg,
        robot2_launch_arg,
        cfg_arg,
        roman_align_node
])

