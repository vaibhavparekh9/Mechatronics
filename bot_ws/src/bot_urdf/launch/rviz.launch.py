import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bot_urdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'bot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    robot_description_param = {'robot_description': robot_description}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param],
        ),
        Node(
            package='bot_urdf',
            executable='cmd_vel_to_odom.py',
            name='cmd_vel_to_odom',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_odom': True,
                'publish_wheel_joint_states': True,
                'wheel_radius': 0.048,
                'track_width': 0.34,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
