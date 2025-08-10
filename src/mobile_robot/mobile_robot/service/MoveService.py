import time

import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.MyNavigationDao import MyNavigationDao
from ..dao.NavigationPtpDao import NavigationPtpDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MoveService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.previous_point = None

        self.__ptp_navigation = NavigationPtpDao(node)
        self.__navigation = MyNavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)

    def rotation_correction(self):
        angle_by_front = self.__radar.get_angle_from_wall(Direction.FRONT)
        angle_by_right = self.__radar.get_angle_from_wall(Direction.RIGHT)
        angle_by_left = self.__radar.get_angle_from_wall(Direction.LEFT)
        angles = [angle_by_front, angle_by_right, angle_by_left]

        min_angle = min(angles, key=lambda x: (abs(x), -x))
        if abs(min_angle) > 1:
            self.__logger.info(f"直角矫正度数 {min_angle}")
            self.rotate(min_angle)

    def lidar_correction(self, distance: float):
        from_wall = self.__radar.get_distance_from_wall(Direction.FRONT)
        i = from_wall - distance
        while abs(i) > 0.05:
            self.line(i)
            from_wall = self.__radar.get_distance_from_wall(Direction.FRONT)
            i = from_wall - distance

    def my_navigation(self, waypoint: str, speed=0.6, block=True):
        self.__navigation.navigation(waypoint, speed)
        if block:
            self.__navigation.wait_finish()

    def navigation(self, path: list[NavigationPoint], speed=0.6, block=True):
        """
        PTP导航到指定点
        :param path: 导航点
        :param speed: 速度
        :param block: 是否阻塞等待完成
        """
        self.__ptp_navigation.navigation(path, speed, speed * 5, False)
        if block:
            self.__ptp_navigation.wait_finish()

    def line(self, distance: float, speed: float = 0.2, is_block=True):
        self.__motion.line(distance, speed)
        time.sleep(1)

        if is_block:
            self.__motion.wait_finish()

    def rotate(self, angle: float, speed: float = 50, is_block=True):
        self.__motion.rotate(angle, speed)
        time.sleep(1)

        if is_block:
            self.__motion.wait_finish()

    def get_status(self):
        return self.__ptp_navigation.get_status()

    def get_my_status(self):
        return self.__navigation.get_status()

    def stop_motion(self):
        self.__motion.stop()

    def stop_my_navigation(self):
        self.__navigation.cancel()
