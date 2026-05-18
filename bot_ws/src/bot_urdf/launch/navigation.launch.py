import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('bot_urdf')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    database_path = LaunchConfiguration('database_path')
    params_file = LaunchConfiguration('params_file')

    default_map = os.path.join(pkg_share, 'maps', 'kitchen.yaml')
    default_db = os.path.join(pkg_share, 'maps', 'rtabmap.db')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to the map .yaml'),
        DeclareLaunchArgument('database_path', default_value=default_db,
                              description='Full path to the RTAB-Map .db file'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to Nav2 params YAML'),

        # --- Map server + lifecycle manager (replaces localization_launch.py minus AMCL) ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        # --- RTAB-Map: visual odometry ---
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
                'publish_tf': False,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera/camera_info'),
                ('depth/image', '/camera/camera/depth/image_raw'),
            ],
        ),

        # --- RTAB-Map: localization mode (replaces AMCL) ---
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
                'database_path': database_path,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                'Reg/Force3DoF': 'true',
                'Grid/FromDepth': 'true',
                'Grid/RangeMax': '5.0',
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '0.4',
                'Optimizer/GravitySigma': '0',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera/camera_info'),
                ('depth/image', '/camera/camera/depth/image_raw'),
            ],
        ),

        # --- Depth to point cloud (for Nav2 obstacle detection) ---
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'decimation': 2,
                'max_depth': 3.0,
                'voxel_size': 0.02,
            }],
            remappings=[
                ('depth/image', '/camera/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/camera/camera_info'),
                ('cloud', '/camera/cloud'),
            ],
        ),

        # --- Segment ground vs obstacles from point cloud ---
        Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            name='obstacles_detection',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '0.4',
                'Grid/NormalsSegmentation': 'false',
            }],
            remappings=[
                ('cloud', '/camera/cloud'),
                ('obstacles', '/camera/obstacles'),
                ('ground', '/camera/ground'),
            ],
        ),

        # --- Nav2 navigation stack ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
    ])
