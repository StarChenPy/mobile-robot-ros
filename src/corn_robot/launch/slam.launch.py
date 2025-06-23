import os

import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'corn_robot'
    package_dir = get_package_share_directory(package_name)

    slam_toolbox_param = DeclareLaunchArgument('slam_toolbox_params_file',
                                               default_value=os.path.join(get_package_share_directory(package_name),
                                                                          'config',
                                                                          'slam_toolbox.yaml'),
                                               description='用于 pub_waypoints 节点的 ROS2 参数文件的完整路径')

    slam_toolbox_node = launch_ros.actions.Node(
        parameters=[LaunchConfiguration('slam_toolbox_params_file')],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    child_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_dir, '/launch', '/urdf.launch.py']))

    return launch.LaunchDescription([
        slam_toolbox_param,
        child_launch,
        slam_toolbox_node
    ])
