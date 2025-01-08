from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('camera'),
        DeclareLaunchArgument('bag'),
        # DeclareLaunchArgument('args', default_value=''),
        DeclareLaunchArgument('uncompress_rgb', default_value='false'),
        # DeclareLaunchArgument('uncompress_depth', default_value='false'),
        DeclareLaunchArgument('color_topic', default_value='color/image_raw'),
        # DeclareLaunchArgument('depth_topic', default_value='aligned_depth_to_color/image_raw'),
    ]

    # ROS 2 rosbag play
    rosbag_play_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag')], #, LaunchConfiguration('args')],
        output='screen'
    )

    # Group for the robot namespace
    transport_nodes = GroupAction([
        DeclareLaunchArgument('robot'),
        PushRosNamespace(LaunchConfiguration('robot')),

        # Group for the camera namespace
        GroupAction([
            DeclareLaunchArgument('camera'),
            PushRosNamespace(LaunchConfiguration('camera')),

            # RGB uncompress node
            Node(
                package='image_transport',
                executable='republish',
                name='rgb_uncompress',
                output='screen',
                arguments=[
                    'compressed',
                    'raw',
                ],
                remappings=[
                    ('in/compressed', [LaunchConfiguration('color_topic'), '/compressed']),
                    ('out', LaunchConfiguration('color_topic'))
                ],
                condition=IfCondition(LaunchConfiguration('uncompress_rgb'))
            ),

            # TODO: add depth uncompress node
            # Depth uncompress node
            # Node(
            #     package='image_transport',
            #     executable='republish',
            #     name='depth_uncompress',
            #     output='screen',
            #     arguments=[
            #         'compressedDepth',
            #         'raw'
            #     ],
            #     condition=IfCondition(LaunchConfiguration('uncompress_depth'))
            # )
        ])
    ])


    return LaunchDescription(
        launch_arguments + [
        rosbag_play_exec,
        transport_nodes
    ])
