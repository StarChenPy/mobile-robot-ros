import rclpy

from ..dao.MotionDao import MotionDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.SensorDao import SensorDao
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.NavigationPoint import NavigationPoint
from ..param.navigation_path import NavPath
from ..util.Singleton import singleton


@singleton
class NavigationService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__navigation = NavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)

    def navigation(self, nav_path: NavPath, speed: float, is_block: bool):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """

        path = []

        for point in nav_path.value:
            if isinstance(point, NavigationPoint):
                path.append(point)
            elif isinstance(point, CorrectivePoint):
                corrective_point1 = NavigationPoint(point.x, point.y, point.yaw1)
                if path:
                    path.append(corrective_point1)
                    self.__navigation.navigation(path, speed, speed * 4)
                    path = []
                    self.__navigation.wait_finish()

                if point.distance1 > 0:
                    self.__sensor.ir_revise(point.distance1)
                else:
                    self.__sensor.ping_revise(-point.distance1)
                self.__sensor.wait_finish()

                self.__odom.init_all(corrective_point1)

                if point.distance2 != 0:
                    corrective_point2 = NavigationPoint(point.x, point.y, point.yaw2)
                    self.__navigation.navigation([corrective_point2], speed, speed * 4)
                    self.__navigation.wait_finish()
                    if point.distance2 > 0:
                        self.__sensor.ir_revise(point.distance2)
                    else:
                        self.__sensor.ping_revise(-point.distance2)
                    self.__sensor.wait_finish()
                    self.__odom.init_all(corrective_point2)
            else:
                self.__logger.error("[导航] 未知导航点!")

        self.__navigation.navigation(path, speed, speed * 4)

        if is_block:
            self.__navigation.wait_finish()

    def init_odom_all(self, point: NavigationPoint):
        self.__odom.init_all(point)

    def line(self, distance: float, speed: float = 0.4, is_block=True):
        self.__motion.line(distance, speed)

        if is_block:
            self.__motion.wait_finish()

    def rotate(self, angle: float, speed: float = 40, is_block=True):
        self.__motion.rotate(angle, speed)

        if is_block:
            self.__motion.wait_finish()

    def get_status(self):
        self.__navigation.get_status()

    def stop(self):
        self.__motion.stop()
        self.__navigation.cancel()
