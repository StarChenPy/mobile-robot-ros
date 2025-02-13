from ..data_type import *

ZERO_POINT = Pose(0, 0, 0)
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
B_MODULE_1_POINT = Pose(1, 0, 0)

START_POINT = Pose(1.1, -2.9, -180)
START_POINT_ENTER = Pose(0.5, -2.9, -180)

GUO_CANG_ENTER_1_POINT = Pose(0.5, -2, 0)
GUO_CANG_ENTER_2_POINT = Pose(1.15, -2, 0)
GUO_CANG_ENTER_3_POINT = Pose(1.15, -1.04, 0)
GUO_CANG_1_POINT = Pose(0.62, -1.04, 90)

GUO_YUAN_ENTER_1_POINT = Pose(0.5, -3.7, 0)
GUO_YUAN_ENTER_2_POINT = Pose(2.05, -3.7, 90)
GUO_YUAN_1_POINT = Pose(2.05, -2.95, 90)


class NavPath(Enum):
    # STARTING_POINT2PICKING_POINT = (STARTING_POINT, TURNING_POINT, PICKING_ENTRANCE)
    #
    # PICKING_CORRIDOR_1 = (PICKING_CORRIDOR_1_START, PICKING_CORRIDOR_1_END)
    B_MODULE_1 = (NavigationPoint(ZERO_POINT), NavigationPoint(B_MODULE_1_POINT))

    B_MODULE_4 = (NavigationPoint(GUO_CANG_1_POINT, Corrective(CorrectiveSensor.PING, 30)),
                  NavigationPoint(GUO_CANG_ENTER_3_POINT), NavigationPoint(GUO_CANG_ENTER_2_POINT),
                  NavigationPoint(GUO_CANG_ENTER_1_POINT), NavigationPoint(START_POINT_ENTER),
                  NavigationPoint(START_POINT))

    B_MODULE_5 = (NavigationPoint(START_POINT, Corrective(CorrectiveSensor.PING, 20)),
                  NavigationPoint(START_POINT_ENTER), NavigationPoint(GUO_CANG_ENTER_1_POINT),
                  NavigationPoint(GUO_CANG_ENTER_2_POINT), NavigationPoint(GUO_CANG_ENTER_3_POINT),
                  NavigationPoint(GUO_CANG_1_POINT))

    B_MODULE_6 = (NavigationPoint(GUO_YUAN_ENTER_2_POINT), NavigationPoint(GUO_YUAN_ENTER_1_POINT),
                  NavigationPoint(START_POINT_ENTER), NavigationPoint(START_POINT))

    B_MODULE_7 = (NavigationPoint(START_POINT, Corrective(CorrectiveSensor.PING, 20)),
                  NavigationPoint(START_POINT_ENTER), NavigationPoint(GUO_YUAN_ENTER_1_POINT),
                  NavigationPoint(GUO_YUAN_ENTER_2_POINT), NavigationPoint(GUO_YUAN_1_POINT))

    B_MODULE_11 = (NavigationPoint(START_POINT, Corrective(CorrectiveSensor.PING, 20)),
                   NavigationPoint(START_POINT_ENTER), NavigationPoint(GUO_CANG_ENTER_1_POINT),
                   NavigationPoint(GUO_CANG_ENTER_2_POINT), NavigationPoint(GUO_CANG_ENTER_3_POINT),
                   NavigationPoint(GUO_CANG_1_POINT),
                   NavigationPoint(GUO_CANG_1_POINT, Corrective(CorrectiveSensor.PING, 30)))

    B_MODULE_12 = (NavigationPoint(GUO_YUAN_ENTER_1_POINT),
                   NavigationPoint(GUO_YUAN_ENTER_1_POINT, Corrective(CorrectiveSensor.PING, 30)),
                   NavigationPoint(GUO_CANG_ENTER_1_POINT), NavigationPoint(GUO_CANG_ENTER_2_POINT),
                   NavigationPoint(GUO_CANG_ENTER_3_POINT), NavigationPoint(GUO_CANG_1_POINT))
