import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory


def generate_launch_description():
    corn_robot_dir = get_package_share_directory('corn_robot')

    with open(os.path.join(corn_robot_dir, 'urdf', 'corn_robot.urdf'), 'r') as infp:
        urdf_content = infp.read()

    robot_state_publisher_node = launch_ros.actions.Node(package="robot_state_publisher",
                                                         executable="robot_state_publisher",
                                                         name="robot_state_publisher",
                                                         parameters=[{"robot_description": urdf_content}])

    joint_state_publisher_node = launch_ros.actions.Node(package="joint_state_publisher",
                                                         executable="joint_state_publisher",
                                                         name="joint_state_publisher")

    return launch.LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
