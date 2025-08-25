# my_navigation_py/launch/navigation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    my_nav_pkg = FindPackageShare(package='my_navigation_py')
    nav2_bringup_pkg = FindPackageShare(package='nav2_bringup')

    # 配置参数
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([my_nav_pkg, 'params', 'nav_params.yaml'])
    )
    map_file = LaunchConfiguration(
        'map_file',
        default=PathJoinSubstitution([my_nav_pkg, 'maps', 'map.yaml'])
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument('params_file', default_value=params_file, description='导航参数文件路径'),
        DeclareLaunchArgument('map_file', default_value=map_file, description='地图文件路径'),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='是否使用仿真时间'),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'bringup_launch.py']),
            launch_arguments={
                'params_file': params_file,
                'map': map_file,
                'use_sim_time': use_sim_time
                
            }.items()
        ),

        Node(
            package='my_navigation_py',
            executable='fixed_nav_service',
            name='fixed_nav_service',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='my_navigation_py',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

    ])
