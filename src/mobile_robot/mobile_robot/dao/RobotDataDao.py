import time

import rclpy

import web_message_transform_ros2.msg
from ..util import Math
from ..util.Singleton import singleton


@singleton
class RobotDataDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__robot_data = None

        node.create_subscription(
            web_message_transform_ros2.msg.RobotData,
            '/web_transform_node/robot_data',
            self.__robot_data_callback,
            10)

    # 读取回调
    def __robot_data_callback(self, msg: web_message_transform_ros2.msg.RobotData):
        self.__robot_data = msg

    def get_robot_data(self) -> web_message_transform_ros2.msg.RobotData:
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        return self.__robot_data

    def get_sonar(self) -> tuple[float, float]:
        sonar0 = []
        sonar1 = []
        for i in range(5):
            sonar = self.get_robot_data().sonar
            sonar0.append(sonar[0])
            sonar1.append(sonar[1])
            time.sleep(0.1)

        return Math.average_without_extremes(sonar0), Math.average_without_extremes(sonar1)

    def get_ir_left(self) -> float:
        return self.__robot_data.ir[0] + 0.128

    def get_ir_right(self) -> float:
        return self.__robot_data.ir[1] + 0.113
