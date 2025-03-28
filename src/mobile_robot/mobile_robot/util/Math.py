import math

import numpy as np

from ..popo.NavigationPoint import NavigationPoint


def calculate_adjacent_side(hypotenuse: float, angle_degrees: float) -> float:
    """
    根据直角三角形斜边长度和斜边与邻边的夹角角度，计算邻边长度

    @param hypotenuse: 直角三角形的斜边长度（必须为正数）
    @param angle_degrees: 斜边与邻边的夹角角度（单位：度，范围 0° < angle < 90°）
    @return float: 与输入角度相邻的直角边长度
    """

    # 将角度从度数转换为弧度，因为math模块的三角函数使用弧度制
    angle_radians = math.radians(angle_degrees)
    # 使用余弦函数计算邻边长度：邻边 = 斜边 × cos(θ)
    adjacent = hypotenuse * math.cos(angle_radians)

    return adjacent


def calculate_right_angle_side(adjacent_length, angle_degrees):
    """
    根据直角边长度和与斜边形成的角度计算另一直角边的长度。

    @param adjacent_length: 直角边的长度（必须为正数）
    @param angle_degrees: 斜边与直角边的夹角（度数，范围0°到90°）
    @return float: 对边的长度
    """

    # 将角度转换为弧度
    angle_radians = math.radians(angle_degrees)
    # 计算另一条直角边（对边）的长度
    opposite_length = adjacent_length * math.tan(angle_radians)
    return opposite_length


def get_target_coordinate(point: NavigationPoint, dis) -> NavigationPoint:
    """
    根据输入的坐标 (x, y) 和角度 yaw（单位：度），以及前进距离 dis，
    计算沿着 yaw 方向前进 dis 距离后得到的新坐标 (new_x, new_y)

    @param point: 坐标点
    @param dis: 前进距离
    @return NavigationPoint: 新的坐标
    """
    # 将角度转换为弧度
    yaw_rad = math.radians(point.yaw)

    # 根据三角函数计算新的坐标
    new_x = point.x + dis * math.cos(yaw_rad)
    new_y = point.y + dis * math.sin(yaw_rad)

    return NavigationPoint(new_x, new_y, point.yaw)


def fit_polar_line_and_get_distance(polar_points: list[tuple[float, float]]):
    """
    给定一组极坐标点 (r, theta)，其中 theta 的单位为度，
    拟合出一条直线，并计算原点 (0,0) 到该直线的垂直距离。

    @param polar_points: list of tuples，每个元组为 (r, theta)，其中 theta 单位为度。
    @return: 原点 (0,0) 到直线的垂直距离
    """
    # 将极坐标转换为直角坐标（输入角度转换为弧度）
    x = np.array([r * np.cos(np.radians(theta)) for r, theta in polar_points])
    y = np.array([r * np.sin(np.radians(theta)) for r, theta in polar_points])

    # 构建设计矩阵，并用最小二乘法拟合直线 y = ax + b
    A = np.vstack([x, np.ones(len(x))]).T
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]

    # 计算原点 (0,0) 到直线的垂直距离：d = |b| / sqrt(a^2 + 1) 并返回
    return np.abs(b) / np.sqrt(a ** 2 + 1)


def fit_polar_line_and_get_angle(polar_points: list[tuple[float, float]]) -> float:
    """
    给定一组极坐标点 (r, theta)，其中 theta 的单位为度，
    拟合出一条直线，并计算直线相对于极坐标0度的角度。

    @param polar_points: list of tuples，每个元组为 (r, theta)，其中 theta 单位为度。
    @return: 直线相对于极坐标0度的角度
    """

    # 将极坐标转换为直角坐标（输入角度转换为弧度）
    x = np.array([r * np.cos(np.radians(theta)) for r, theta in polar_points])
    y = np.array([r * np.sin(np.radians(theta)) for r, theta in polar_points])

    # 构建设计矩阵，并用最小二乘法拟合直线 y = ax + b
    A = np.vstack([x, np.ones(len(x))]).T
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]

    # 计算直线与x轴正方向的夹角，然后转换为角度并返回
    return np.degrees(np.arctan(a))


def is_behind(point1: NavigationPoint, point2: NavigationPoint, angle_threshold: float) -> bool:
    """
    判断 B 是否在 A 的后方

    @param point1: NavigationPoint，包含坐标 (x, y) 和朝向角 yaw（单位：度）
    @param point2: NavigationPoint，包含坐标 (x, y) 和朝向角 yaw（可以忽略）
    @param angle_threshold: 判断阈值
    @return: True 如果 B 在 A 的后面，否则 False
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


def point_to_point(point1: NavigationPoint, point2: NavigationPoint, dis: float) -> list[NavigationPoint]:
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
        return [point1]  # 两点重合，返回起点

    step_count = math.floor(L / dis)  # 计算最大步数
    points = []

    # 计算路径与 y(ros) 轴的夹角
    yaw = math.atan2(dy, dx)

    for k in range(step_count + 1):
        ratio = (k * dis) / L
        x = point1.x + dx * ratio
        y = point1.y + dy * ratio
        points.append(NavigationPoint(x, y, math.degrees(yaw)))  # 转换为角度存储

    points.pop(0)

    return points


def average_without_extremes(lst: list[float]) -> float:
    """
    去掉一个最高值和一个最低值，返回剩下数的平均值
    """
    if len(lst) <= 2:
        return 0.0
    sorted_lst = sorted(lst)
    trimmed = sorted_lst[1:-1]
    return sum(trimmed) / len(trimmed)


def distance_from_origin(x1, y1, x2, y2):
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
