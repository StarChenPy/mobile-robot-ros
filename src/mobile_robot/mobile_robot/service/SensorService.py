import time

import rclpy

from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..util.Singleton import singleton


@singleton
class SensorService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__sensor = SensorDao(node)
        self.__robot_data = RobotDataDao(node)

    def ping_revise(self, dis: float, is_block):
        self.__sensor.ping_revise(dis)
        if is_block:
            time.sleep(1)
            self.__sensor.wait_finish()

    def ir_revise(self, dis: float, is_block):
        self.__sensor.ir_revise(dis)
        if is_block:
            time.sleep(1)
            self.__sensor.wait_finish()

    def get_ir_claws(self) -> float:
        return self.__robot_data.get_robot_data().ir[0]

    def get_ir_front(self) -> float:
        return self.__robot_data.get_robot_data().ir[1]