import math

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