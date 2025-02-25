import rclpy

from ..param.navigation_path import NavPath
from ..popo.NavigationPoint import NavigationPoint
from ..service.NavigationService import NavigationService
from ..util.Singleton import singleton


@singleton
class NavigationController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__navigation = NavigationService(node)

    def navigation(self, nav_path: NavPath, speed=0.4, is_block=True):
        self.__navigation.navigation(nav_path, speed, is_block)

    def init_pose(self, point: NavigationPoint):
        self.__navigation.init_odom_all(point)

    def rotate(self, angle):
        self.__navigation.rotate(angle)
