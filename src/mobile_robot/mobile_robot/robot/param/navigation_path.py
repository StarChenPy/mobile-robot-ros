from dataclasses import dataclass
from enum import Enum


@dataclass
class Point:
    x: float
    y: float
    yaw: float


class NavPoint(Enum):
    # 出发点
    STARTING_POINT = Point(0, 0, 0)
    # 第一个转弯点
    TURNING_POINT = Point(2.5, 0, 90)
    # 采摘入口
    PICKING_ENTRANCE = Point(2.5, -3.8, 180)
    # 采摘走廊 1
    PICKING_CORRIDOR_1_START = Point(1.15, -3.8, 90)
    PICKING_CORRIDOR_1_MIDDLE = Point(2.0, -1.35, -180)
    PICKING_CORRIDOR_1_END = Point(1.15, -1.3, 90)
    # 采摘走廊 2
    PICKING_CORRIDOR_2_START = Point(1.35, -0.35, -90)
    PICKING_CORRIDOR_2_MIDDLE = Point(1.3, -1.35, -90)
    PICKING_CORRIDOR_2_END = Point(1.35, -3.5, -90)
    # 采摘走廊 3
    PICKING_CORRIDOR_3_START = Point(0.0, -3.5, 90)
    PICKING_CORRIDOR_3_END = Point(0.0, -0.35, 90)


class NavPath(Enum):
    STARTING_POINT2TURNING_POINT = (NavPoint.STARTING_POINT, NavPoint.TURNING_POINT)