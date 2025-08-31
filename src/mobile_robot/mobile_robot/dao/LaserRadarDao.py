import math
import time

import numpy as np
import rclpy
import rclpy.qos
from sensor_msgs.msg import LaserScan

from corn_robot_toolbox.util.MedianFilter import MedianFilter
from ..popo.Direction import Direction
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton

RADAR_ERROR_LEFT = 2.38
RADAR_ERROR_FRONT = 1.98
RADAR_ERROR_RIGHT = 2.16


@singleton
class LaserRadarDao:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()
        self.node = node

        self.radar_data = LaserScan()

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        node.create_subscription(LaserScan, '/scan', self.__callback, qos_profile)

    def __callback(self, msg: LaserScan):
        self.radar_data = msg

    def get_radar_data(self, target_angle: float) -> tuple[float, float]:
        """
        获取与目标角度最接近的激光测距数据，使用 NumPy 优化处理速度。

        参数：
            target_angle (float): 目标角度（单位：度）

        返回：
            tuple[float, float]: 距离（米）与实际角度（度）
        """
        if self.radar_data is None:
            return 0.0, 0.0

        # 转换并筛选有效数据（非零）
        ranges = np.array(self.radar_data.ranges)
        valid_mask = ranges > 0
        if not np.any(valid_mask):
            return 0.0, 0.0

        # 构造角度数组，与 ranges 一一对应
        angle_min = self.radar_data.angle_min
        angle_increment = self.radar_data.angle_increment
        indices = np.arange(len(ranges))
        angles = angle_min + indices * angle_increment

        # 提取有效数据
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # 计算与目标角度最接近的数据索引
        target_rad = math.radians(target_angle)
        diffs = np.abs(valid_angles - target_rad)
        best_idx = np.argmin(diffs)

        # 提取并转换为标准 float 类型
        best_range = float(valid_ranges[best_idx])
        best_angle = float(math.degrees(valid_angles[best_idx]))
        return best_range, best_angle

    def __get_radar_points(self, direction: Direction, scan_angle) -> list[tuple[float, float]]:
        points = []
        start_angle = 0

        if direction == Direction.RIGHT:
            start_angle = 0 - (scan_angle / 2)
        elif direction == Direction.FRONT:
            start_angle = 90 - (scan_angle / 2)
        elif direction == Direction.LEFT:
            start_angle = 180 - (scan_angle / 2)

        rclpy.spin_once(self.node)
        for i in range(scan_angle * 2):
            angle = Math.normalize_angle(start_angle + (i * 0.5))
            points.append(self.get_radar_data(angle))

        return points

    def get_angle_from_wall(self, direction: Direction, scan_angle=30, secondary_confirmation=False) -> float:
        angle_list = []

        # 角度滤波器
        angle_filter = MedianFilter(window_size=5) # 能快速去掉瞬时尖峰，反应比较灵敏

        for i in range(5):
            points = self.__get_radar_points(direction, scan_angle)
            angle = Math.fit_polar_line_and_get_angle(points)

            # 中值滤波
            angle = angle_filter.update(angle)

            # 补偿逻辑...
            if direction == Direction.FRONT:
                angle += RADAR_ERROR_FRONT
            else:
                if angle > 0:
                    angle -= 90
                else:
                    angle += 90

                if direction == Direction.LEFT:
                    angle += RADAR_ERROR_LEFT
                elif direction == Direction.RIGHT:
                    angle += RADAR_ERROR_RIGHT

            angle_list.append(angle)
            time.sleep(0.2)

        extremes = Math.average_without_extremes(angle_list)

        self.logger.debug(f"{direction.name} 测量角度 {extremes}")

        if not secondary_confirmation and extremes >= 10:
            self.logger.debug(f"{direction.name} 测量角度较大，二次确认")
            sc = self.get_angle_from_wall(direction, scan_angle, True)
            if abs(Math.normalize_angle(extremes - sc)) > 3:
                self.logger.warn(f"{direction.name} 二次确认未通过, 返回0")
                return 0

        return extremes

    def get_distance_from_wall(self, direction: Direction, scan_angle=30) -> float:
        return self.get_distance_and_angle(direction, scan_angle)[1]

    def get_distance_and_angle(self, direction: Direction, scan_angle=30) -> tuple[float, float]:
        distance_list = []
        distance_filter = MedianFilter(window_size=5) # 能快速去掉瞬时尖峰，反应比较灵敏
        angle_from_wall = self.get_angle_from_wall(direction)

        for i in range(5):
            points = self.__get_radar_points(direction, scan_angle)
            distance = Math.fit_polar_line_and_get_distance(points)

            distance = distance_filter.update(distance)

            if direction == Direction.FRONT:
                distance += 0.2
            else:
                side = Math.calculate_right_angle_side(0.2, abs(angle_from_wall))
                if direction == Direction.LEFT:
                    side = -side
                if angle_from_wall > 0:
                    distance += side
                else:
                    distance -= side

            distance_list.append(distance)

        # 方差过大，说明扫出墙壁
        var = np.var(distance_list)
        if var > 0.1:
            self.logger.warn(f"{distance_list} 方差 {var} 过大")
            return angle_from_wall, -1

        extremes = Math.average_without_extremes(distance_list)
        self.logger.debug(f"{direction.name} 测量距离 {extremes}")
        return angle_from_wall, extremes
