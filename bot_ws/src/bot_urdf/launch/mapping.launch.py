from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera/camera_info'),
                ('depth/image', '/camera/camera/depth/image_raw'),
            ],
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'approx_sync': True,
                'Grid/FromDepth': 'true',
                'Grid/RangeMax': '5.0',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera/camera_info'),
                ('depth/image', '/camera/camera/depth/image_raw'),
            ],
        ),
    ])
