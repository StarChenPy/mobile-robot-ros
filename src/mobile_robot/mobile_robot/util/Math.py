import math

import numpy as np

from ..popo.NavigationPoint import NavigationPoint
from ..popo.Point import Point
from ..popo.Rectangle import Rectangle


def ease_in_out_interp(start, end, steps):
    """非线性插值函数：Ease-In-Out"""
    x = np.linspace(0, 1, steps)
    t = 0.5 * (1 - np.cos(np.pi * x))  # cosine-based缓动曲线
    return start + (end - start) * t


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


def calculate_right_triangle_angle(a: float, b: float) -> float:
    """
    计算直角三角形中斜边与边 b 的夹角（单位：度）
    参数:
        a (float): 一条直角边，与角相对
        b (float): 另一条直角边，作为角的邻边
    返回:
        float: 斜边与 b 的夹角，单位为度
    """
    if b == 0:
        raise ValueError("b 不能为 0，因为不能除以 0")

    radians = math.atan2(a, b)  # atan2 自动处理正负情况
    degrees = math.degrees(radians)
    return degrees


def calculate_hypotenuse(a: float, b: float) -> float:
    """
    计算直角三角形的斜边长度。

    参数：
    a -- 第一条直角边长度
    b -- 第二条直角边长度

    返回：
    斜边的长度
    """
    return math.sqrt(a ** 2 + b ** 2)


def get_point_angle(a, b):
    """
    计算从点 A 到点 B 的方向角，角度以 A 为基准。
    坐标系：x 轴朝上，y 轴朝左。

    参数:
        a: 点 A 的坐标，格式为 (x1, y1)
        b: 点 B 的坐标，格式为 (x2, y2)

    返回:
        角度（float），范围 [0, 360)，单位为度。
    """
    dx = b[0] - a[0]
    dy = b[1] - a[1]

    # 坐标轴变换：x 朝上、y 朝左 → 相当于将标准坐标系旋转了 -90°
    angle_rad = math.atan2(-dy, dx)
    angle_deg = math.degrees(angle_rad)

    # 转换到 0~360 度
    if angle_deg < 0:
        angle_deg += 360

    return normalize_angle(angle_deg)


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
    return (np.abs(b) / np.sqrt(a ** 2 + 1)).item()


def polar_to_cartesian(polar_point: tuple[float, float]) -> tuple[float, float]:
    # 将极坐标转换为直角坐标（输入角度转换为弧度）
    r, theta = polar_point
    x = r * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(theta))

    return x, y


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
    return np.degrees(np.arctan(a)).item()


def is_behind(point1: NavigationPoint, point2: NavigationPoint, angle_threshold: float) -> bool:
    """
    判断 B 是否在 A 的后方

    @param point1: NavigationPoint，包含坐标 (x, y) 和朝向角 yaw（单位：度）
    @param point2: NavigationPoint，包含坐标 (x, y) 和朝向角 yaw（可以忽略）
    @param angle_threshold: 判断阈值
    @return: True 如果 B 在 A 的后面，否则 False
    """
    if not point1 or not point2:
        return False

    if not point1.yaw or not point2.yaw:
        return False

    dx = point2.x - point1.x
    dy = point2.y - point1.y

    yaw = abs(point1.yaw - point2.yaw)
    if yaw > 180:
        yaw = abs(yaw - 360)
    if yaw > angle_threshold * 2:
        return False

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


def pixel_to_horizontal_distance_x_centered(x_pixel: float, camera_height: float) -> float:
    """
    将图像x像素坐标（以图像中心为0）转换为现实世界的横向距离（米）。

    输入的 x_pixel 是以图像中心为0，向右为正，向左为负。
    相机垂直向下安装，图像宽度640px，水平FOV为110°。
    """
    fov_deg = 80
    image_width = 600
    fov_rad = math.radians(fov_deg)

    # 焦距（像素单位）
    focal_length = image_width / (2 * math.tan(fov_rad / 2))

    # 计算角度
    angle_rad = math.atan2(x_pixel, focal_length)

    # 计算横向现实距离
    distance = camera_height * math.tan(angle_rad)
    return distance


def pixel_to_distance_from_bottom(y_pixel: float, camera_height: float) -> float:
    """
    将图像中的像素位置映射为现实世界距离，底部为 0 米，向上递增。

    参数:
    - y_pixel: 像素坐标（图像底部为0）
    - camera_height: 相机离地高度（米）

    返回:
    - 从图像底部到该点的现实距离（米）
    """
    IMAGE_HEIGHT = 320  # 图像高度（像素）
    VERTICAL_FOV_DEG = 55  # 垂直视场角（度）

    # 像素相对图像中心的位置
    y_pixel = IMAGE_HEIGHT - y_pixel
    pixel_from_center = y_pixel - IMAGE_HEIGHT / 2

    # 每像素的角度（弧度）
    angle_per_pixel = math.radians(VERTICAL_FOV_DEG) / IMAGE_HEIGHT
    angle = pixel_from_center * angle_per_pixel

    # 当前点的距离
    distance = camera_height * math.tan(angle)

    # 底部的距离作为 0 基准
    angle_min = -math.radians(VERTICAL_FOV_DEG) / 2
    distance_bottom = camera_height * math.tan(angle_min)

    # 相对底部的距离
    return distance - distance_bottom


def pixel_to_distance_from_center(y_pixel: float, camera_height: float) -> float:
    """
    将图像中的像素位置映射为现实世界距离，底部为 0 米，向上递增。

    参数:
    - y_pixel: 像素坐标（图像底部为0）
    - camera_height: 相机离地高度（米）

    返回:
    - 从图像底部到该点的现实距离（米）
    """
    IMAGE_HEIGHT = 320  # 图像高度（像素）
    VERTICAL_FOV_DEG = 55  # 垂直视场角（度）

    # 像素相对图像中心的位置
    y_pixel = IMAGE_HEIGHT - y_pixel
    pixel_from_center = y_pixel - IMAGE_HEIGHT / 2

    # 每像素的角度（弧度）
    angle_per_pixel = math.radians(VERTICAL_FOV_DEG) / IMAGE_HEIGHT
    angle = pixel_from_center * angle_per_pixel

    # 当前点的距离
    distance = camera_height * math.tan(angle)

    return distance


def pixel_to_world(point: Point, dis):
    """
    将图像像素坐标转换为相对于图像底部中心 (320, 480) 的地面平面坐标。
    参数：
        point: tuple(int, int)，图像像素坐标 (x, y)
        dis: float，相机到地面的垂直高度（单位：米）
        img_width: 图像宽度（默认 640）
        img_height: 图像高度（默认 480）
        h_fov_deg: 水平视角（默认 110°）
        v_fov_deg: 垂直视角（默认 55°）
    返回：
        (x_m, y_m): 地面坐标（单位：米），以图像底部中心为原点，x 向右为正，y 向下为正
    """

    img_width = 640
    img_height = 480
    h_fov_deg = 110
    v_fov_deg = 55

    # 图像底部中心点
    cx = img_width / 2
    cy = img_height  # 底部为 y=480

    # 像素偏移
    dx = point.x - cx
    dy = cy - point.y  # 注意是向上的为正角度，因此底部为0，向上为负

    # 每像素对应的视角（弧度）
    angle_per_pixel_x = math.radians(h_fov_deg) / img_width
    angle_per_pixel_y = math.radians(v_fov_deg) / img_height

    # 像素偏移角度
    angle_x = dx * angle_per_pixel_x
    angle_y = dy * angle_per_pixel_y

    # 地面坐标偏移（单位：米）
    x_m = math.tan(angle_x) * dis
    y_m = math.tan(angle_y) * dis

    return Point(x_m, y_m)


def split_rectangle(x1, y1, x2, y2, cols=6, rows=3) -> dict[int: list[Rectangle]]:
    # 生成横向和纵向的分割点
    x_edges = np.linspace(x1, x2, cols + 1)
    y_edges = np.linspace(y1, y2, rows + 1)

    rectangles = {}

    # 遍历每一个小矩形区域
    for i in range(rows):
        rectangles[i] = []
        for j in range(cols):
            rect = Rectangle(x_edges[j].item(), y_edges[i].item(), x_edges[j + 1].item(), y_edges[i + 1].item())
            rectangles[i].append(rect)

    return rectangles


def find_points_in_squares(points: list[Point], squares: list[Rectangle]):
    """
    找出每个正方形中包含的坐标点
    :param points: 坐标点列表，格式 [(x1, y1), (x2, y2), ...]
    :param squares: 正方形列表，每个正方形用左上角和右下角表示，格式 [((x_min1, y_min1), (x_max1, y_max1)), ...]
    :return: 字典，键为正方形索引，值为该正方形包含的点列表
    """
    results = []
    for idx, square in enumerate(squares):
        for point in points:
            if square.x1 <= point.x <= square.x2 and square.y1 <= point.y <= square.y2:
                results.append(point)
                points.remove(point)  # 确保每个点只被计入一次
    return results


def round_right_angle(i: float) -> int:
    """
    给定一个角度，向直角逼近
    """

    f = 1 if i > 0 else -1

    if abs(i % (90 * f)) >= 45:
        return  90 * f * (abs(int(i / 90)) + 1)
    else:
        return 90 * f * (abs(int(i / 90)))


def normalize_angle(angle_deg: float) -> float:
    """将角度标准化到 [-180, 180]"""
    if angle_deg >= 180:
        angle_deg -= 360
    elif angle_deg < -180:
        angle_deg += 360
    return angle_deg
