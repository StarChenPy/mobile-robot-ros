import time

import rclpy

from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.NavigationPoint import NavigationPoint
from ..service.NavigationService import NavigationService
from ..util.Singleton import singleton


@singleton
class MoveController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__navigation = NavigationService(node)

    def navigation(self, nav_path: list[NavigationPoint or CorrectivePoint], speed=0.4, is_block=True):
        self.__navigation.navigation(nav_path, speed, is_block)
        time.sleep(1)

    def init_pose(self, point: NavigationPoint):
        self.__navigation.init_odom_all(point)

    def reset_yaw(self, yaw: float):
        self.__navigation.init_odom_yaw(yaw)

    def rotate(self, angle):
        self.__navigation.rotate(angle)
