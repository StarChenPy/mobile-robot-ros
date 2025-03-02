import math

def calculate_adjacent_side(hypotenuse, angle_degrees):
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