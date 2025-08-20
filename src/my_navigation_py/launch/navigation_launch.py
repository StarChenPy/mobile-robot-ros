import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取导航功能包的路径
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    launch_dir = os.path.join(nav2_dir, 'launch')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    
    # 定义参数文件路径（默认使用nav2的默认参数）
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml']
    )
    
    # 包含nav2的默认启动文件，并添加话题重映射
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            # 重映射cmd_vel话题
            'cmd_vel_remap': '/chassis/cmd_vel'
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'
        ),
        nav2_launch
    ])
    