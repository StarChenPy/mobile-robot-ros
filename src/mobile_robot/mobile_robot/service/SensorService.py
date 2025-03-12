import time

import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..util.Math import Math
from ..util.Singleton import singleton


@singleton
class SensorService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__sensor = SensorDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)
        self.__odom = OdomDao(node)

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

    def __get_radar_points(self, direction: Direction):
        points = []
        start_angle = 0

        if direction == Direction.RIGHT:
            start_angle = 0
        elif direction == Direction.FRONT:
            start_angle = 80
        elif direction == Direction.LEFT:
            start_angle = 155

        for i in range(0, 5):
            angle = start_angle + 5 * i
            dis = self.__radar.get_radar_data(angle)
            points.append((dis, angle))

        return points

    def get_distance_from_wall(self, direction: Direction):
        if direction == Direction.BACK:
            self.__logger.error("[传感器] 无法获取距离: 不支持的方向")
            return 0

        # 返回距离雷达扫描的5个坐标拟合成的直线的垂直距离
        points = self.__get_radar_points(direction)
        distance = Math.fit_polar_line_and_get_distance(points)

        if direction == Direction.FRONT:
            distance += 0.225
        else:
            # 补偿因倾斜导致的雷达与墙和机器人中心与墙的距离不一致的问题
            angle_from_wall = self.get_angle_from_wall(direction)
            side = Math.calculate_dui_side(0.225, abs(angle_from_wall))

            if direction == Direction.LEFT:
                side = -side

            if angle_from_wall > 0:
                distance += side
            else:
                distance -= side

        return distance

    def get_angle_from_wall(self, direction: Direction) -> float:
        if direction == Direction.BACK:
            self.__logger.error("[传感器] 无法获取角度: 不支持的方向")
            return 0

        points = self.__get_radar_points(direction)
        angle_relative_to_x = Math.fit_polar_line_and_get_angle(points)

        if direction == Direction.FRONT:
            pass
        else:
            if angle_relative_to_x < 0:
                angle_relative_to_x += 90
            else:
                angle_relative_to_x -= 90

        return angle_relative_to_x

    def get_ir_claws(self) -> float:
        return self.__robot_data.get_robot_data().ir[0]

    def get_ir_front(self) -> float:
        return self.__robot_data.get_robot_data().ir[1]

    def get_radar_data(self, target_angle: float) -> float:
        return self.__radar.get_radar_data(target_angle)

    def init_odom_all(self, point: NavigationPoint):
        self.__odom.init_all(point)
        time.sleep(1)

    def init_location(self, x, y):
        self.__odom.init_location(x, y)

    def init_odom_yaw(self, yaw):
        self.__odom.init_yaw(yaw)