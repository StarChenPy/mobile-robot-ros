import time

import rclpy

from ..dao.MotionDao import MotionDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.SensorDao import SensorDao
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.NavigationPoint import NavigationPoint
from ..util.Singleton import singleton


@singleton
class NavigationService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__navigation = NavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)

    def navigation(self, nav_path: list[NavigationPoint or CorrectivePoint], speed: float, is_block: bool):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """
        path = []

        for point in nav_path:
            if isinstance(point, NavigationPoint):
                path.append(point)
            elif isinstance(point, CorrectivePoint):
                self.__navigation_corrective(path, point, speed)
                path = []
            else:
                self.__logger.error("[导航] 未知导航点!")

        self.__navigation.navigation(path, speed, speed * 4)

        if is_block:
            self.__navigation.wait_finish()

    def __navigation_corrective(self, path, point, speed):
        corrective_point = NavigationPoint(point.x, point.y, point.yaw1)

        if path:
            path.append(corrective_point)
            self.__navigation.navigation(path, speed, speed * 4)
            self.__navigation.wait_finish()
        elif self.__odom.get_init():
            self.__navigation.navigation([corrective_point], speed, speed * 4)
            self.__navigation.wait_finish()

        if point.distance1 > 0:
            self.__sensor.ir_revise(point.distance1)
        else:
            self.__sensor.ping_revise(-point.distance1)

        time.sleep(1)
        self.__sensor.wait_finish()
        self.__odom.init_all(corrective_point)

        if point.distance2 != 0:
            corrective_point.yaw = point.yaw2
            self.__navigation.navigation([corrective_point], speed, speed * 4)
            self.__navigation.wait_finish()

            if point.distance2 > 0:
                self.__sensor.ir_revise(point.distance2)
            else:
                self.__sensor.ping_revise(-point.distance2)

            time.sleep(1)
            self.__sensor.wait_finish()
            self.__odom.init_all(corrective_point)

    def init_odom_all(self, point: NavigationPoint):
        self.__odom.init_all(point)

    def init_odom_yaw(self, yaw):
        self.__odom.init_yaw(yaw)

    def line(self, distance: float, speed: float = 0.4, is_block=True):
        self.__motion.line(distance, speed)

        if is_block:
            self.__motion.wait_finish()

    def rotate(self, angle: float, speed: float = 40, is_block=True):
        self.__motion.rotate(angle, speed)

        if is_block:
            self.__motion.wait_finish()

    def get_status(self):
        return self.__navigation.get_status()

    def stop_motion(self):
        self.__motion.stop()

    def stop_navigation(self):
        self.__navigation.cancel()
