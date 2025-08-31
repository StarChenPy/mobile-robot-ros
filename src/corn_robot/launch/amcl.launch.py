import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    package_name = 'corn_robot'
    package_dir = get_package_share_directory(package_name)

    # 参数声明
    amcl_params = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=os.path.join(package_dir, 'config', 'amcl.yaml'),
        description='用于 amcl 的 ROS2 参数文件的完整路径'
    )

    # 使用LifecycleNode替代普通Node
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': os.path.join(package_dir, 'maps', 'map.yaml'),
            'use_sim_time': False
        }],
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('amcl_params_file')],
    )

    positioning = Node(
        package=package_name,
        executable='positioning',
        name='positioning',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('amcl_params_file')],
    )

    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
            {"bond_timeout": 20.0}
        ]
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'urdf.launch.py'))
    )

    return LaunchDescription([
        amcl_params,
        urdf_launch,
        map_server,
        amcl,
        positioning,
        lifecycle_manager
    ])