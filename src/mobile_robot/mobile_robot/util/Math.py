import math

import numpy as np

from ..popo.NavigationPoint import NavigationPoint


class Math:
    @classmethod
    def calculate_adjacent_side(cls, hypotenuse, angle_degrees):
        """
        根据斜边长度和与斜边形成的角度计算邻边的长度。

        参数：
            hypotenuse (float): 斜边的长度（必须为正数）
            angle_degrees (float): 斜边与邻边的夹角（度数，范围0°到90°）

        返回：
            float: 邻边的长度
        """
        # 将角度转换为弧度，因为math.cos函数使用弧度作为输入
        angle_radians = math.radians(angle_degrees)

        # 使用余弦函数计算邻边长度：邻边 = 斜边 * cos(角度)
        adjacent = hypotenuse * math.cos(angle_radians)

        return adjacent

    @classmethod
    def calculate_dui_side(cls, hypotenuse, angle_degrees):
        """
        根据斜边长度和与斜边形成的角度计算对边的长度。

        参数：
            hypotenuse (float): 斜边的长度（必须为正数）
            angle_degrees (float): 斜边与邻边的夹角（度数，范围0°到90°）

        返回：
            float: 对边的长度
        """
        # 将角度转换为弧度，因为math.cos函数使用弧度作为输入
        angle_radians = math.radians(angle_degrees)

        # 使用正函数计算对边长度：对边 = 斜边 * sin(角度)
        adjacent = hypotenuse * math.sin(angle_radians)

        return adjacent

    @classmethod
    def get_target_coordinate(cls, point: NavigationPoint, dis) -> NavigationPoint:
        """
        根据输入的坐标 (x, y) 和角度 yaw（单位：度），以及前进距离 dis，
        计算沿着 yaw 方向前进 dis 距离后得到的新坐标 (new_x, new_y)

        参数：
            x, y      : 原始坐标
            yaw_deg   : 朝向角度（单位：度）
            dis       : 前进距离

        返回：
            new_x, new_y : 新的坐标
        """
        # 将角度转换为弧度
        yaw_rad = math.radians(point.yaw)

        # 根据三角函数计算新的坐标
        new_x = point.x + dis * math.cos(yaw_rad)
        new_y = point.y + dis * math.sin(yaw_rad)

        return NavigationPoint(new_x, new_y, point.yaw)

    @classmethod
    def fit_polar_line_and_get_distance(cls, polar_points: list[tuple[float, float]]):
        """
        给定一组极坐标点 (r, theta)，其中 theta 的单位为度，
        拟合出一条直线，并计算原点 (0,0) 到该直线的垂直距离。

        参数：
            polar_points: list of tuples，每个元组为 (r, theta)，其中 theta 单位为度。

        返回：原点 (0,0) 到直线的垂直距离
        """
        # 将极坐标转换为直角坐标（输入角度转换为弧度）
        x = np.array([r * np.cos(np.radians(theta)) for r, theta in polar_points])
        y = np.array([r * np.sin(np.radians(theta)) for r, theta in polar_points])

        # 构建设计矩阵，并用最小二乘法拟合直线 y = ax + b
        A = np.vstack([x, np.ones(len(x))]).T
        a, b = np.linalg.lstsq(A, y, rcond=None)[0]

        # 计算原点 (0,0) 到直线的垂直距离：d = |b| / sqrt(a^2 + 1) 并返回
        return np.abs(b) / np.sqrt(a**2 + 1)

    @classmethod
    def fit_polar_line_and_get_angle(cls, polar_points: list[tuple[float, float]]) -> float:
        """
        给定一组极坐标点 (r, theta)，其中 theta 的单位为度，
        拟合出一条直线，并计算直线相对于极坐标0度的角度。

        参数：
            polar_points: list of tuples，每个元组为 (r, theta)，其中 theta 单位为度。

        返回：直线相对于极坐标0度的角度
        """

        # 将极坐标转换为直角坐标（输入角度转换为弧度）
        x = np.array([r * np.cos(np.radians(theta)) for r, theta in polar_points])
        y = np.array([r * np.sin(np.radians(theta)) for r, theta in polar_points])

        # 构建设计矩阵，并用最小二乘法拟合直线 y = ax + b
        A = np.vstack([x, np.ones(len(x))]).T
        a, b = np.linalg.lstsq(A, y, rcond=None)[0]

        # 计算直线与x轴正方向的夹角，然后转换为角度并返回
        return np.degrees(np.arctan(a))

    @classmethod
    def is_behind(cls, point1: NavigationPoint, point2: NavigationPoint, angle_threshold: float) -> bool:
        """
        判断 B 是否在 A 的后方

        :param point1: NavigationPoint，包含坐标 (x, y) 和朝向角 yaw（单位：度）
        :param point2: NavigationPoint，包含坐标 (x, y) 和朝向角 yaw（可以忽略）
        :param angle_threshold: 判断阈值
        :return: True 如果 B 在 A 的后面，否则 False
        """
        dx = point2.x - point1.x
        dy = point2.y - point1.y

        # 处理两点重合的情况
        if dx == 0 and dy == 0:
            return False

        # 计算x2相对于x1的方向角度
        theta_rad = math.atan2(dy, dx)
        theta_deg = math.degrees(theta_rad)

        # 计算yaw1的反方向并规范化到[-180, 180)
        opposite_angle = (point1.yaw + 180) % 360
        if opposite_angle > 180:
            opposite_angle -= 360

        # 计算角度差并规范化到[-180, 180)
        delta = theta_deg - opposite_angle
        delta = (delta + 180) % 360 - 180

        return abs(delta) <= angle_threshold

    @classmethod
    def point_to_point(cls, point1: NavigationPoint, point2: NavigationPoint, dis: float):
        """
        给定点1和点2，根据给定的距离分割路径
        @param point1 路径点1
        @param point2 路径点2
        @param dis 步进距离
        """
        if dis <= 0:
            raise ValueError("间距必须大于0")

        dx = point2.x - point1.x
        dy = point2.y - point1.y
        L = math.hypot(dx, dy)  # 计算两点之间的距离

        if L == 0:
            return point1  # 两点重合，返回起点

        step_count = math.floor(L / dis)  # 计算最大步数
        points = []

        for k in range(step_count + 1):
            ratio = (k * dis) / L
            x = point1.x + dx * ratio
            y = point1.y + dy * ratio
            points.append(NavigationPoint(x, y, point2.yaw))

        points.pop(0)

        return points

    @classmethod
    def average_without_extremes(cls, lst: list[float]) -> float:
        """
        去掉一个最高值和一个最低值，返回剩下数的平均值
        """
        if len(lst) <= 2:
            return 0.0
        sorted_lst = sorted(lst)
        trimmed = sorted_lst[1:-1]
        return sum(trimmed) / len(trimmed)

    @classmethod
    def distance_from_origin(cls, x1, y1, x2, y2):
        """
        计算原点到由 (x1, y1) 和 (x2, y2) 所确定直线的垂直距离。

        利用三角形面积公式：
          面积 = |x1*y2 - x2*y1| / 2
        与底边长度（两点间距离）：
          底边 = sqrt((x2-x1)^2 + (y2-y1)^2)
        则垂直距离 d = (2 * 面积) / 底边
                          = |x1*y2 - x2*y1| / sqrt((x2-x1)^2 + (y2-y1)^2)

        参数:
          x1, y1: 第一个点的坐标
          x2, y2: 第二个点的坐标

        返回:
          原点到直线的垂直距离。

        注意: 当两个点重合时，函数将抛出异常，因为无法确定一条直线。
        """
        numerator = abs(x1 * y2 - x2 * y1)
        denominator = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        if denominator == 0:
            raise ValueError("两个点不能重合，必须确定一条直线。")

        return numerator / denominator
