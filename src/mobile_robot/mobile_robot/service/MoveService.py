import time

import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.NavigationPoint import NavigationPoint
from ..util.Math import Math
from ..util.Singleton import singleton


@singleton
class MoveService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__navigation = NavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)

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

    def along_left_wall(self, travel_distance: float, distance_from_wall: float, speed=0.4, is_block=True):
        """
        沿左墙行驶
        """
        odom_backup = self.__robot_data.get_robot_data().odom

        radar_data = self.__radar.get_radar_data(90)
        self.__odom.init_all(NavigationPoint(0, radar_data, 0))
        self.__navigation.navigation([NavigationPoint(travel_distance, distance_from_wall, 0)], speed, is_block)

        while rclpy.ok() and self.__navigation.get_status():
            odom_data = self.__robot_data.get_robot_data().odom
            radar_data = self.__radar.get_radar_data(90)
            self.__odom.init_location(odom_data.x, radar_data)

        new_point = Math.get_target_coordinate(odom_backup, travel_distance)
        self.__odom.init_all(new_point)

    def along_right_wall(self, travel_distance: float, distance_from_wall: float, speed=0.4, is_block=True):
        """
        沿右墙行驶
        """
        odom_backup = self.__robot_data.get_robot_data().odom

        radar_data = self.__radar.get_radar_data(-90)
        self.__odom.init_all(NavigationPoint(0, radar_data, 0))
        self.__navigation.navigation([NavigationPoint(travel_distance, distance_from_wall, 0)], speed, is_block)

        while rclpy.ok() and self.__navigation.get_status():
            odom_data = self.__robot_data.get_robot_data().odom
            radar_data = self.__radar.get_radar_data(-90)
            self.__odom.init_location(odom_data.x, radar_data)

        new_point = Math.get_target_coordinate(odom_backup, travel_distance)
        self.__odom.init_all(new_point)
