import time

import rclpy

from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.NavigationPoint import NavigationPoint
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..util.Singleton import singleton


@singleton
class MoveController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__navigation = MoveService(node)
        self.__sensor = SensorService(node)

    def navigation(self, nav_path: list[NavigationPoint or CorrectivePoint], speed=0.4, is_block=True):
        self.__navigation.navigation(nav_path, speed, is_block)
        time.sleep(0.5)

    def init_pose(self, point: NavigationPoint):
        self.__sensor.init_odom_all(point)

    def init_location(self, x, y):
        self.__sensor.init_location(x, y)

    def reset_yaw(self, yaw: float):
        self.__sensor.init_odom_yaw(yaw)

    def rotate(self, angle):
        self.__navigation.rotate(angle)

    def ping_revise(self, dis: float, is_block=True):
        self.__sensor.ping_revise(dis, is_block)

    def ir_revise(self, dis: float, is_block=True):
        self.__sensor.ir_revise(dis, is_block)
