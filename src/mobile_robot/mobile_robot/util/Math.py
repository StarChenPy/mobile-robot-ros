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
