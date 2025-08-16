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

RADAR_ERROR_LEFT = -2.68
RADAR_ERROR_FRONT = -2.3
RADAR_ERROR_RIGHT = -2.5


@singleton
class LaserRadarDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__radar_data = LaserScan()

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        node.create_subscription(LaserScan, '/scan', self.__callback, qos_profile)

    def __callback(self, msg: LaserScan):
        self.__radar_data = msg

    def get_radar_data(self, target_angle: float) -> tuple[float, float]:
        """
        获取与目标角度最接近的激光测距数据，使用 NumPy 优化处理速度。

        参数：
            target_angle (float): 目标角度（单位：度）

        返回：
            tuple[float, float]: 距离（米）与实际角度（度）
        """
        if self.__radar_data is None:
            return 0.0, 0.0

        # 转换并筛选有效数据（非零）
        ranges = np.array(self.__radar_data.ranges)
        valid_mask = ranges > 0
        if not np.any(valid_mask):
            return 0.0, 0.0

        # 构造角度数组，与 ranges 一一对应
        angle_min = self.__radar_data.angle_min
        angle_increment = self.__radar_data.angle_increment
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

    def __get_radar_points(self, direction: Direction) -> list[tuple[float, float]]:
        points = []
        start_angle = 0

        if direction == Direction.RIGHT:
            start_angle = -30
        elif direction == Direction.FRONT:
            start_angle = 75
        elif direction == Direction.LEFT:
            start_angle = 180

        rclpy.spin_once(self.__node)
        for i in range(1, 30):
            angle = Math.normalize_angle(start_angle + i)
            points.append(self.get_radar_data(angle))

        return points

    def get_angle_from_wall_once(self, direction: Direction) -> float:
        angle_list = []

        for i in range(5):
            points = self.__get_radar_points(direction)
            angle = Math.fit_polar_line_and_get_angle(points)

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
            time.sleep(0.3)

        self.__logger.debug(f"{direction.name} 扫描到的雷达角度为 {angle_list}")

        return Math.average_without_extremes(angle_list)

    def get_angle_from_wall(self, direction: Direction) -> float:
        if direction == Direction.BACK:
            self.__logger.error("无法获取角度: 不支持的方向")
            return 0

        for i in range(5):
            angle_1 = self.get_angle_from_wall_once(direction)
            time.sleep(0.2)
            angle_2 = self.get_angle_from_wall_once(direction)

            if abs(angle_1 - angle_2) < 0.5:
                angle = (angle_1 + angle_2) / 2
                self.__logger.debug(f"扫描到的雷达角度为 {angle}")
                return angle
            else:
                self.__logger.warn(f"雷达两次角度获取误差较大 {abs(angle_2 - angle_1)}，重试 {i + 1} 次")
        return 0

    def get_distance_from_wall_once(self, direction: Direction) -> float:
        # 返回距离雷达扫描的5个坐标拟合成的直线的垂直距离
        points = self.__get_radar_points(direction)
        distance = Math.fit_polar_line_and_get_distance(points)

        # 方差过大，说明扫出墙壁
        var = np.var(distance)
        if var > 0.1:
            self.__logger.warn(f"{distance} 方差 {var} 过大")
            return 0

        if direction == Direction.FRONT:
            # 加上从雷达到机器人中心的距离
            distance += 0.2
        else:
            # 补偿因倾斜导致的雷达与墙和机器人中心与墙的距离不一致的问题
            angle_from_wall = self.get_angle_from_wall(direction)

            side = Math.calculate_right_angle_side(0.2, abs(angle_from_wall))

            if direction == Direction.LEFT:
                side = -side

            if angle_from_wall > 0:
                distance += side
            else:
                distance -= side

        return distance

    def get_distance_from_wall(self, direction: Direction) -> float:
        if direction == Direction.BACK:
            self.__logger.error("无法获取距离: 不支持的方向")
            return 0

        dis = 0
        for i in range(5):
            dis = self.get_distance_from_wall_once(direction)
            if dis > 0:
                break
            self.__logger.warn(f"获取 {direction.name} 方向雷达距离失败, 重试 {i} 次")
            time.sleep(0.2)

        if dis != 0:
            self.__logger.debug(f"{direction.name} 扫描到的雷达距离为 {dis}")
        else:
            self.__logger.error(f"雷达无法获取 {direction.name} 方向的距离!")

        # 左、右墙可能有误差，加一下
        if direction == Direction.LEFT:
            dis -= 0.0
        elif direction == Direction.RIGHT:
            dis -= 0.00

        return dis
