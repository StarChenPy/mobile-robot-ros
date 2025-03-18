import time

import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..util.Math import Math
from ..util.Singleton import singleton


@singleton
class MoveService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__navigation = NavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)

    def navigation(self, nav_path: list[NavigationPoint], speed: float, is_block: bool):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """
        path = []
        buffer = None

        for point in nav_path:
            if isinstance(point, CorrectivePoint):
                if path:
                    path.append(point)
                    self.__navigation.navigation(path, speed, speed * 5, 3, 3, False)
                    self.__navigation.wait_finish()
                elif self.__odom.get_init():
                    self.__navigation.navigation([point], speed, speed * 5, 3, 3, False)
                    self.__navigation.wait_finish()
                else:
                    self.__odom.init_yaw(point.yaw)

                time.sleep(1)
                self.corrective(point)
                path = []
                continue

            if buffer is None:
                odom = self.__robot_data.get_robot_data().odom
                buffer = NavigationPoint(odom.x, odom.y, odom.w)

            if buffer.yaw is not None and Math.is_behind(buffer, point, 45):
                # 如果这个点位在上个点位的后面，就倒车回去
                if path:
                    self.__navigation.navigation(path, speed, speed * 5, 3, 3, False)
                    self.__navigation.wait_finish()
                    path = []
                if point.yaw is None:
                    point.yaw = buffer.yaw
                self.__navigation.navigation([point], speed, speed * 5, 3, 3, True)
                self.__navigation.wait_finish()
            else:
                path.append(point)

            buffer = point

        if path:
            self.__navigation.navigation(path, speed, speed * 5, 3, 3, False)

        if is_block:
            self.__navigation.wait_finish()

    def corrective(self, point: CorrectivePoint):
        x_buffer = 0
        y_buffer = 0
        angle_from_wall = 0
        for corrective in point.corrective_data:
            match corrective.direction:
                case Direction.FRONT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                    x_buffer = distance_from_wall - corrective.distance
                case Direction.BACK:
                    sonar = self.__robot_data.get_sonar()
                    distance_from_wall = Math.distance_from_origin(-5, sonar[0], 5, sonar[1]) + 0.222
                    x_buffer = distance_from_wall - corrective.distance
                case Direction.LEFT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                    y_buffer = distance_from_wall - corrective.distance
                case Direction.RIGHT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                    y_buffer = distance_from_wall - corrective.distance

        if point.yaw == 0:
            # 验证可用
            self.__odom.init_location(point.x + x_buffer, point.y + y_buffer)
        elif point.yaw == 90:
            # 验证可用
            self.__odom.init_location(point.x + y_buffer, point.y + x_buffer)
        elif point.yaw == 180 or point.yaw == -180:
            # 未验证可用
            self.__odom.init_location(point.x + x_buffer, point.y + y_buffer)
        elif point.yaw == -90:
            # 验证可用
            self.__odom.init_location(point.x + y_buffer, point.y - x_buffer)

        if angle_from_wall != 0:
            self.__odom.init_yaw(point.yaw - angle_from_wall)

        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)

    def line(self, distance: float, speed: float = 0.4, is_block=True):
        self.__motion.line(distance, speed)

        if is_block:
            self.__motion.wait_finish()

    def rotate(self, angle: float, speed: float = 50, is_block=True):
        self.__motion.rotate(angle, speed)

        if is_block:
            self.__motion.wait_finish()

    def get_status(self):
        return self.__navigation.get_status()

    def stop_motion(self):
        self.__motion.stop()

    def stop_navigation(self):
        self.__navigation.cancel()
