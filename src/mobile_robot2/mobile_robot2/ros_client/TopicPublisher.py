from py_trees_ros.publishers import FromBlackboard
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from web_message_transform_ros2.msg import RobotCtrl

robot_ctrl_pub = FromBlackboard("Robot Ctrl Publishers",
                                "/web_transform_node/robot_ctrl",
                                RobotCtrl,
                                QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10),
                                "robot_ctrl")
