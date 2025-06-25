import time

import rclpy

from web_message_transform_ros2.msg import Pose
from ..dao.InitialPoseDao import InitialPoseDao
from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
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
        self.__initial_pose = InitialPoseDao(node)

    def ping_revise(self, dis: float, is_block):
        self.__sensor.ping_revise(dis)
        if is_block:
            time.sleep(1)
            self.__sensor.wait_finish()

    def ir_revise(self, dis: float, is_block):
        self.__sensor.ir_revise(dis)
        if is_block:
            time.sleep(1)
            self.__sensor.wait_finish()

    def get_distance_from_wall(self, direction: Direction) -> float:
        return self.__radar.get_distance_from_wall(direction)

    def get_angle_from_wall(self, direction: Direction) -> float:
        return self.__radar.get_angle_from_wall(direction)

    def get_ir_left(self) -> float:
        return self.__robot_data.get_ir_left()

    def get_ir_right(self) -> float:
        return self.__robot_data.get_ir_right()

    def get_sonar(self) -> tuple[float, float]:
        return self.__robot_data.get_sonar()

    def get_radar_data(self, target_angle: float) -> tuple[float, float]:
        return self.__radar.get_radar_data(target_angle)

    def get_odom_data(self) -> Pose:
        rclpy.spin_once(self.__node)
        return self.__robot_data.get_robot_data().odom

    def initial_pose(self, point: NavigationPoint):
        self.__initial_pose.set_initial_pose(point)

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

    def reset_odom(self):
        self.__odom.set_init(False)
