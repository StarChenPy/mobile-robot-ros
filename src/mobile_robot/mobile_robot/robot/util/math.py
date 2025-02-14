from .data_type import Rectangle


def calculate_rectangle_center(rectangle: Rectangle):
    center_x = (rectangle.x1 + rectangle.x2) / 2
    center_y = (rectangle.y1 + rectangle.y2) / 2
    return center_x, center_y