import time

import rclpy

from ..dao.InitialPoseDao import InitialPoseDao
from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.MyNavigationDao import MyNavigationDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.NavigationPoint import NavigationPoint
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MoveService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__navigation = NavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)
        self.__my_navigation = MyNavigationDao(node)
        self.init_pose = InitialPoseDao(node)

    def __navigation_handle(self, path: list[NavigationPoint], speed: float, is_block: bool):
        ROTATION_ACCELERATION = 3
        ROTATION_DECELERATION = 3
        self.__navigation.navigation(path, speed, speed * 5, ROTATION_ACCELERATION, ROTATION_DECELERATION, False)
        if is_block:
            self.__navigation.wait_finish()

    def my_navigation(self, waypoint_name: str, speed=0.4, is_block=True):
        self.__my_navigation.navigation(waypoint_name, speed)

        if is_block:
            self.__my_navigation.wait_finish()

    def navigation(self, nav_path: list[NavigationPoint], speed=0.4, is_block=True):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """
        path = []
        previous_point = None

        for point in nav_path:
            if not self.init_pose.get_init():
                self.init_pose.set_initial_pose(point)

            if previous_point is None:
                odom = self.__robot_data.get_robot_data().odom
                previous_point = NavigationPoint(odom.x, odom.y, odom.w)

            if previous_point.yaw is not None and Math.is_behind(previous_point, point, 30):
                # 如果这个点位在上个点位的后面，就倒车回去
                if path:
                    self.__navigation_handle(path, speed, True)
                    path = []
                if point.yaw is None:
                    point.yaw = previous_point.yaw
                self.__navigation.navigation([point], speed, speed * 5, 3, 3, True)
                self.__navigation.wait_finish()
            else:
                path.append(point)

            previous_point = point

        if path:
            self.__navigation_handle(path, speed, is_block)

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
        return self.__navigation.get_status()

    def stop_motion(self):
        self.__motion.stop()

    def stop_navigation(self):
        self.__navigation.cancel()
