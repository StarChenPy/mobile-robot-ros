from dataclasses import dataclass

from ..popo.Point import Point


@dataclass
class Rectangle:
    x1: int
    y1: int
    x2: int
    y2: int

    def get_area(self) -> float:
        """
        获取矩形面积
        """
        length = abs(self.x2 - self.x1)
        width = abs(self.y2 - self.y1)
        return length * width

    def get_perimeter(self) -> float:
        """
        获取矩形周长
        """
        length = abs(self.x2 - self.x1)
        width = abs(self.y2 - self.y1)
        return 2 * (length + width)

    def get_rectangle_center(self) -> Point:
        """
        获取矩形中心点
        """
        center_x = (self.x1 + self.x2) / 2
        center_y = (self.y1 + self.y2) / 2
        return Point(center_x, center_y)
