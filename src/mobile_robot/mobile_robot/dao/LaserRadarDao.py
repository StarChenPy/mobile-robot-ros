import math
import time

import numpy as np
import rclpy
import rclpy.qos
from sensor_msgs.msg import LaserScan

from ..popo.Direction import Direction
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton

RADAR_ERROR = 2.4


@singleton
class LaserRadarDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__radar_data = LaserScan()

        qos_profile = rclpy.qos.QoSProfile(reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,depth = 10)
        node.create_subscription(LaserScan, '/scan', self.__callback, qos_profile)

    def __callback(self, msg: LaserScan):
        self.__radar_data = msg

    def get_radar_data(self, target_angle: float) -> tuple[float, float]:
        """
        获取雷达与某物体在某角度下的距离
        @param target_angle: 目标角度
        @return 距离与实际角度
        """
        rclpy.spin_once(self.__node)
        # 检查雷达数据是否存在
        if self.__radar_data is None:
            return 0, 0

        # 转换目标角度为弧度
        target_angle_rad = math.radians(target_angle)
        angle_min = self.__radar_data.angle_min
        angle_increment = self.__radar_data.angle_increment

        best_range = None
        best_index = None
        best_diff = float('inf')  # 初始设为无穷大

        for i, range_value in enumerate(self.__radar_data.ranges):
            # 跳过无效测量（0表示无效）
            if range_value == 0:
                continue

            # 计算当前数据点的角度
            current_angle = angle_min + i * angle_increment
            # 计算与目标角度的差值
            diff = abs(current_angle - target_angle_rad)
            # 如果当前角度更接近目标角度，则更新
            if diff < best_diff:
                best_diff = diff
                best_range = range_value
                best_index = i

        # 如果没有找到有效数据，则返回0
        if best_index is None:
            return 0, 0

        best_angle = angle_min + best_index * angle_increment
        return best_range, math.degrees(best_angle)

    def __get_radar_points(self, direction: Direction) -> list[tuple[float, float]]:
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
            points.append(self.get_radar_data(angle))

        return points

    def get_angle_from_wall_once(self, direction: Direction) -> float:
        points = self.__get_radar_points(direction)
        angle = Math.fit_polar_line_and_get_angle(points)

        if direction == Direction.FRONT:
            pass
        else:
            if angle < 0:
                angle += 90
            else:
                angle -= 90

        return angle

    def get_angle_from_wall(self, direction: Direction) -> float:
        if direction == Direction.BACK:
            self.__logger.error("无法获取角度: 不支持的方向")
            return 0

        angle_list = []

        for i in range(5):
            once_angle = self.get_angle_from_wall_once(direction)
            angle_list.append(once_angle)
            time.sleep(0.2)

        if angle_list.count(0) > 1:
            self.__logger.warn(f"过多不可信角度")
            return 0

        self.__logger.debug(f"扫描到的雷达角度为 {angle_list}")

        return Math.average_without_extremes(angle_list) - RADAR_ERROR

    def get_distance_from_wall_once(self, direction: Direction) -> float or None:
        # 返回距离雷达扫描的5个坐标拟合成的直线的垂直距离
        points = self.__get_radar_points(direction)
        distance = Math.fit_polar_line_and_get_distance(points)

        if direction == Direction.FRONT:
            # 加上从雷达到机器人中心的距离
            distance += 0.225
        else:
            # 补偿因倾斜导致的雷达与墙和机器人中心与墙的距离不一致的问题
            angle_from_wall = self.get_angle_from_wall_once(direction)
            if angle_from_wall == 0:
                self.__logger.warn("雷达距离数据无效")
                return None

            side = Math.calculate_right_angle_side(0.225, abs(angle_from_wall))

            if direction == Direction.LEFT:
                side = -side

            if angle_from_wall > 0:
                distance += side
            else:
                distance -= side

        return distance

    def get_distance_from_wall(self, direction: Direction) -> float | None:
        if direction == Direction.BACK:
            self.__logger.error("无法获取距离: 不支持的方向")
            return 0

        distance_list = []

        for i in range(5):
            once_dis = self.get_distance_from_wall_once(direction)
            distance_list.append(once_dis)
            time.sleep(0.2)

        if distance_list.count(None) > 1:
            self.__logger.warn(f"过多不可信距离")
            return None

        if None in distance_list:
            distance_list.remove(None)

        # 方差过大，说明扫出墙壁
        var = np.var(distance_list)
        if var > 0.1:
            self.__logger.warn(f"{distance_list} 方差 {var} 过大")
            return None

        self.__logger.debug(f"扫描到的雷达距离为 {distance_list}")

        # 返回平均值
        dis = Math.average_without_extremes(distance_list)

        # 左、右墙有大约1cm度误差，加一下
        if direction == Direction.LEFT:
            dis += 0.01
        elif direction == Direction.RIGHT:
            dis -= 0.01
        return dis
