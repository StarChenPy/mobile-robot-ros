import time

import rclpy

from web_message_transform_ros2.msg import Pose
from ..dao.CorrectionOdomDao import CorrectionOdomDao
from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class SensorService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__sensor = SensorDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)
        self.__odom = OdomDao(node)
        self.__correction = CorrectionOdomDao(node)
        self.__motion = MotionDao(node)

    def ping_revise(self, dis: float, is_block=True):
        dis = (dis - 0.225) * 100
        self.__sensor.ping_revise(dis)
        self.__logger.info("开始超声修正")
        if is_block:
            time.sleep(1)
            self.__sensor.wait_finish()

    def ir_revise(self, dis: float, is_block):
        self.__sensor.ir_revise(dis)
        if is_block:
            time.sleep(1)
            self.__sensor.wait_finish()

    def lidar_revise(self, dis: float, is_block=True):
        from_wall = self.__radar.get_distance_from_wall(Direction.FRONT, 10)

        if not from_wall:
            self.__logger.warn("雷达没有可用值，重试一次")
            from_wall = self.__radar.get_distance_from_wall(Direction.FRONT, 10)
            if not from_wall:
                self.__logger.warn("雷达没有可用值，跳过对前矫正")
                return

        self.__logger.info(f"雷达对前矫正, 距墙 {from_wall} 米, 目标 {dis} 米, 移动 {from_wall - dis} 米")
        self.__motion.line(from_wall - dis, 0.2)
        if is_block:
            time.sleep(1)
            self.__motion.wait_finish()

    def get_lidar_data(self, start: int, end: int) -> list[tuple[float, float]]:
        l = []
        if start > end:
            start, end = end, start
        for i in range(start, end):
            l.append(self.__radar.get_radar_data(Math.normalize_angle(i)))
        return l

    def get_distance_from_wall(self, direction: Direction) -> float:
        return self.__radar.get_distance_from_wall(direction)

    def get_angle_from_wall(self, direction: Direction) -> float:
        return self.__radar.get_angle_from_wall(direction)

    def get_ir_left(self) -> float:
        return self.__robot_data.get_ir_left()

    def get_ir_right(self) -> float:
        return self.__robot_data.get_ir_right()

    def get_sonar(self) -> float:
        sonar = self.__robot_data.get_sonar()
        distance_from_wall = Math.distance_from_origin(-5, sonar[0], 5, sonar[1]) + 0.226
        return distance_from_wall

    def get_odom_data(self) -> Pose:
        rclpy.spin_once(self.__node)
        return self.__robot_data.get_robot_data().odom

    def init_odom_all(self, point: NavigationPoint):
        self.__odom.init_all(point)
        time.sleep(1)
        rclpy.spin_once(self.__node)

    def init_location(self, x, y):
        self.__odom.init_location(x, y)
        time.sleep(1)
        rclpy.spin_once(self.__node)

    def init_odom_yaw(self, yaw):
        self.__odom.init_yaw(yaw)
        time.sleep(1)
        rclpy.spin_once(self.__node)

    def correction(self, waypoint_name: str, block=True):
        self.__correction.send_correction_odom(waypoint_name)
        if block:
            self.__correction.wait_finish()

    def wait_correction_finish(self):
        self.__correction.wait_finish()
