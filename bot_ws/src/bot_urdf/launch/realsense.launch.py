from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'device_type': 'd435',
                'enable_color': True,
                'rgb_camera.color_profile': '640,480,30',
                'enable_depth': True,
                'depth_module.depth_profile': '640x480x15',
                'pointcloud.enable': True,
                'enable_sync': True,
                'align_depth.enable': True,
            }],
        ),
    ])
