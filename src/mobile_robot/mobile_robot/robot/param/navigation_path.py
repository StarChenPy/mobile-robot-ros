from ..data_type import *


class NavPoint(Enum):
    # 出发点
    STARTING_POINT = NavigationPoint(Pose(0, 0, 0))

    # # 省赛地图
    # # 第一个转弯点
    # TURNING_POINT = Pose(2.3, 0, 90)
    # # 采摘入口
    # PICKING_ENTRANCE = Pose(2.3, -3.7, 180)
    # # 采摘走廊 1
    # PICKING_CORRIDOR_1_START = Pose(1.3, -3.7, 90)
    # PICKING_CORRIDOR_1_MIDDLE = Pose(1.3, -2.0, -180)
    # PICKING_CORRIDOR_1_END = Pose(1.3, -1.3, 90)
    # # 采摘走廊 2
    # PICKING_CORRIDOR_2_START = Pose(1.35, -0.35, -90)
    # PICKING_CORRIDOR_2_MIDDLE = Pose(1.3, -1.35, -90)
    # PICKING_CORRIDOR_2_END = Pose(1.35, -3.5, -90)
    # # 采摘走廊 3
    # PICKING_CORRIDOR_3_START = Pose(0.0, -3.5, 90)
    # PICKING_CORRIDOR_3_END = Pose(0.0, -0.35, 90)

    # 市赛地图
    B_MODULE_1_POINT = NavigationPoint(Pose(0.25, 0, 0))

    START_POINT = NavigationPoint(Pose(1.2, -2.9, -180))
    START_POINT_ENTER = NavigationPoint(Pose(0.5, -2.9, -180))

    GUO_CANG_ENTER_1_POINT = NavigationPoint(Pose(0.5, -2, 90))
    GUO_CANG_ENTER_2_POINT = NavigationPoint(Pose(1.2, -2, 0))
    GUO_CANG_ENTER_3_POINT = NavigationPoint(Pose(1.2, -1.04, 0))

    GUO_CANG_1_POINT = NavigationPoint(Pose(0.62, -1.04, 90), Corrective(CorrectiveSensor.PING, 30))


class NavPath(Enum):
    # STARTING_POINT2PICKING_POINT = (NavPoint.STARTING_POINT, NavPoint.TURNING_POINT, NavPoint.PICKING_ENTRANCE)
    #
    # PICKING_CORRIDOR_1 = (NavPoint.PICKING_CORRIDOR_1_START, NavPoint.PICKING_CORRIDOR_1_END)
    B_MODULE_1 = (NavPoint.STARTING_POINT, NavPoint.B_MODULE_1_POINT)
    B_MODULE_4 = (NavPoint.GUO_CANG_1_POINT, NavPoint.GUO_CANG_ENTER_3_POINT, NavPoint.GUO_CANG_ENTER_2_POINT,
                  NavPoint.GUO_CANG_ENTER_1_POINT, NavPoint.START_POINT_ENTER, NavPoint.START_POINT)
