import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bot_urdf')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_path = os.path.join(pkg_share, 'urdf', 'bot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')

    # Gazebo converts package:// URIs to model:// when parsing URDF.
    # Register parent dirs of both bot_urdf and realsense2_description so
    # Gazebo can resolve model://bot_urdf/meshes/... and
    # model://realsense2_description/meshes/d435.dae
    pkg_realsense_desc = get_package_share_directory('realsense2_description')
    gazebo_model_dirs = [
        os.path.join(pkg_share, os.pardir),
        os.path.join(pkg_realsense_desc, os.pardir),
    ]
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = ':'.join(gazebo_model_dirs) + (':' + existing if existing else '')

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
        output='screen',
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
