import time

import rclpy.node

from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RobotDataDao import RobotDataDao
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class RobotService:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()
        self.__node = node

        self.__robot_data = RobotDataDao(node)
        self.__robot_ctrl = RobotCtrlDao(node)

    def with_robot_connect(self):
        self.logger.info("等待机器人链接...")
        while self.__robot_data.get_robot_data() is None:
            time.sleep(0.5)
        return

    def with_start_button(self):
        self.logger.info("正在等待开始按钮按下...")
        while self.__robot_data.get_robot_data().di[1]:
            rclpy.spin_once(self.__node)
            time.sleep(0.1)
        self.set_start_led(True)

    def set_start_led(self, state: bool):
        for _ in range(20):
            self.__robot_ctrl.write_do(0, state)
