import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
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
        parameters=[LaunchConfiguration('amcl_params_file')],
    )

    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
            {'bond_timeout': 120.0},  # 增加到120秒
            {'heartbeat_timeout': 60.0},
            {'enable_bond_heartbeat': True}
        ]
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'urdf.launch.py'))
    )

    # 事件处理器 - 确保启动顺序
    map_server_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=map_server,
            on_start=[
                LogInfo(msg='Map server started, waiting for activation...'),
                TimerAction(
                    period=5.0,  # 等待5秒让map_server完全启动
                    actions=[lifecycle_manager]
                )
            ]
        )
    )

    return LaunchDescription([
        amcl_params,
        urdf_launch,
        map_server,
        amcl,
        map_server_event,  # 添加事件处理器

        # 调试信息
        LogInfo(msg='Starting AMCL localization system...'),
    ])