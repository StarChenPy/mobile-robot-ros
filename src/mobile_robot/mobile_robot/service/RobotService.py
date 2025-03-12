import time

import rclpy.node

from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RobotDataDao import RobotDataDao
from ..util.Singleton import singleton


@singleton
class RobotService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__robot_data = RobotDataDao(node)
        self.__robot_ctrl = RobotCtrlDao(node)

    def with_robot_connect(self):
        if self.__robot_data.get_robot_data() is not None:
            return

    def start_button(self):
        while self.__robot_data.get_robot_data().di[1]:
            rclpy.spin_once(self.__node)
            time.sleep(0.1)
        self.__logger.info("[机器人] 开始按钮被按下")
        self.start_led(True)

    def start_led(self, state: bool):
        self.__robot_ctrl.write_do(0, state)
