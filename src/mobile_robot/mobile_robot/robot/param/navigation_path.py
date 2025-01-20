from ..data_type import *


class NavPoint(Enum):
    # 出发点
    STARTING_POINT = Pose(0, 0, 0)
    # 第一个转弯点
    TURNING_POINT = Pose(2.3, 0, 90)
    # 采摘入口
    PICKING_ENTRANCE = Pose(2.3, -3.7, 180)
    # 采摘走廊 1
    PICKING_CORRIDOR_1_START = Pose(1.3, -3.7, 90)
    PICKING_CORRIDOR_1_MIDDLE = Pose(1.3, -2.0, -180)
    PICKING_CORRIDOR_1_END = Pose(1.3, -1.5, 90)
    # 采摘走廊 2
    PICKING_CORRIDOR_2_START = Pose(1.35, -0.35, -90)
    PICKING_CORRIDOR_2_MIDDLE = Pose(1.3, -1.35, -90)
    PICKING_CORRIDOR_2_END = Pose(1.35, -3.5, -90)
    # 采摘走廊 3
    PICKING_CORRIDOR_3_START = Pose(0.0, -3.5, 90)
    PICKING_CORRIDOR_3_END = Pose(0.0, -0.35, 90)


class NavPath(Enum):
    STARTING_POINT2PICKING_POINT = (NavPoint.STARTING_POINT, NavPoint.TURNING_POINT, NavPoint.PICKING_ENTRANCE)

    PICKING_CORRIDOR_1 = (NavPoint.PICKING_CORRIDOR_1_START, NavPoint.PICKING_CORRIDOR_1_END)
