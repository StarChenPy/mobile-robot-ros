from py_trees_ros.subscribers import ToBlackboard
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan
from web_message_transform_ros2.msg import RobotData


lidar_data_sub = ToBlackboard("Lidar Data Subscriber",
                              '/scan',
                              LaserScan,
                              QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10),
                              {"lidar_data/raw": None})


robot_data_sub = ToBlackboard("Robot Data Subscriber",
                              "/web_transform_node/robot_data",
                              RobotData,
                              QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10),
                              {"robot_data/raw": None})
