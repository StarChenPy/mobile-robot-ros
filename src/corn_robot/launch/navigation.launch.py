import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'corn_robot'
    package_dir = get_package_share_directory(package_name)

    navigation_params = DeclareLaunchArgument('navigation_params_file',
                                              default_value=os.path.join(package_dir, 'config', 'navigation.yaml'),
                                              description='用于 navigation.launch 的 ROS2 参数文件的完整路径')

    pub_waypoints_node = Node(package=package_name,
                              executable='pub_waypoints',
                              name='pub_waypoints',
                              parameters=[LaunchConfiguration('navigation_params_file')])

    generate_path_node = Node(package=package_name,
                              executable='generate_path',
                              name='generate_path',
                              parameters=[LaunchConfiguration('navigation_params_file')])

    robot_motion_service = Node(package=package_name,
                                executable='robot_motion',
                                name='corn_robot_motion',
                                parameters=[LaunchConfiguration('navigation_params_file')])

    navigation_to_waypoint_node = Node(package=package_name,
                                       executable='navigation_to_waypoint',
                                       name='navigation_to_waypoint',
                                       parameters=[LaunchConfiguration('navigation_params_file')])

    amcl_launch_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'amcl.launch.py')))

    return LaunchDescription([
        navigation_params,
        pub_waypoints_node,
        generate_path_node,
        robot_motion_service,
        navigation_to_waypoint_node,
        amcl_launch_path
    ])
