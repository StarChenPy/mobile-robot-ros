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

RADAR_ERROR = 2.7


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
            start_angle = -10
        elif direction == Direction.FRONT:
            start_angle = 80
        elif direction == Direction.LEFT:
            start_angle = 170

        for i in range(1, 6):
            angle = start_angle + 5 * i
            if angle > 180:
                angle -= 360
            if angle < -180:
                angle += 360
            points.append(self.get_radar_data(angle))

        return points

    def get_angle_from_wall_once(self, direction: Direction) -> float:
        points = self.__get_radar_points(direction)
        angle = Math.fit_polar_line_and_get_angle(points)

        if direction == Direction.FRONT:
            pass
        elif direction == Direction.LEFT:
            if angle < 0:
                angle += 90
        else:
            if angle < 0:
                angle += 90
            else:
                angle -= 90

        return angle - RADAR_ERROR

    def get_angle_from_wall(self, direction: Direction) -> float:
        if direction == Direction.BACK:
            self.__logger.error("无法获取角度: 不支持的方向")
            return 0

        for i in range(1, 11):
            angle_1 = self.get_angle_from_wall_once(direction)
            time.sleep(0.2)
            angle_2 = self.get_angle_from_wall_once(direction)

            if abs(angle_2 - angle_1) < 0.5:
                angle = (angle_1 + angle_2) / 2
                self.__logger.debug(f"扫描到的雷达角度为 {angle}")
                return angle
            else:
                self.__logger.warn(f"雷达两次角度获取误差较大 {abs(angle_2 - angle_1)}，重试 {i} 次")
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
            distance += 0.21
        else:
            # 补偿因倾斜导致的雷达与墙和机器人中心与墙的距离不一致的问题
            angle_from_wall = self.get_angle_from_wall(direction)

            side = Math.calculate_right_angle_side(0.21, abs(angle_from_wall))

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
            self.__logger.warn(f"获取 {direction} 方向雷达距离失败, 重试 {i} 次")
            time.sleep(0.2)

        if dis != 0:
            self.__logger.debug(f"{direction} 扫描到的雷达距离为 {dis}")
        else:
            self.__logger.error(f"雷达无法获取 {direction} 方向的距离!")

        # 左、右墙有误差，加一下
        if direction == Direction.LEFT:
            dis -= 0.0
        elif direction == Direction.RIGHT:
            dis -= 0.00

        return dis
