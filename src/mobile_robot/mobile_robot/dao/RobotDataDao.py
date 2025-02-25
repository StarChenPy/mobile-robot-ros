import rclpy

import web_message_transform_ros2.msg
from ..util.Singleton import singleton


@singleton
class RobotDataDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.__robot_data = web_message_transform_ros2.msg.RobotData()

        node.create_subscription(
            web_message_transform_ros2.msg.RobotData,
            '/web_transform_node/robot_data',
            self.__robot_data_callback,
            10)

    # 读取回调
    def __robot_data_callback(self, msg):
        self.__robot_data = msg

    def get_robot_data(self) -> web_message_transform_ros2.msg.RobotData:
        return self.__robot_data