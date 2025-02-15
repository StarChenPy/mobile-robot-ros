import enum

from ..util.data_type import Pose, NavigationPoint, Corrective, CorrectiveSensor

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
B_MODULE_1_POINT = Pose(1.05, 0, 0)

START_POINT = Pose(1.1, -2.9, -180)
START_ENTER_POINT = Pose(0.5, -2.9, -180)

WAREHOUSE_ENTER_1_POINT = Pose(0.5, -2, 0)
WAREHOUSE_ENTER_2_POINT = Pose(1.15, -2, 0)
WAREHOUSE_ENTER_3_POINT = Pose(1.15, -1.04, 0)
WAREHOUSE_1_POINT = Pose(0.62, -1.04, 90)

# 果园坐标
ORCHARD_ENTER_POINT = Pose(0.5, -3.7, 0)
ORCHARD_CORRIDOR_ENTER_1_POINT = Pose(2, -3.7, 90)
ORCHARD_CORRIDOR_EXIT_1_POINT = Pose(2, -0.4, -90)

ORCHARD_CORRIDOR_ENTER_2_POINT = Pose(2.8, -3.7, 90)
ORCHARD_CORRIDOR_EXIT_2_POINT = Pose(2.8, -0.4, -90)

ORCHARD_CORRIDOR_ENTER_3_POINT = Pose(3.6, -3.7, 90)
ORCHARD_CORRIDOR_EXIT_3_POINT = Pose(3.6, -0.4, -90)


# 果仓1号点
ORCHARD_1_POINT = Pose(2, -2.92, 90)


class NavPath(enum.Enum):
    # STARTING_POINT2PICKING_POINT = (STARTING_POINT, TURNING_POINT, PICKING_ENTRANCE)
    #
    # PICKING_CORRIDOR_1 = (PICKING_CORRIDOR_1_START, PICKING_CORRIDOR_1_END)
    B_MODULE_1 = (NavigationPoint(B_MODULE_1_POINT), )

    B_MODULE_4 = (NavigationPoint(WAREHOUSE_1_POINT, Corrective(CorrectiveSensor.PING, 30)),
                  NavigationPoint(WAREHOUSE_ENTER_3_POINT), NavigationPoint(WAREHOUSE_ENTER_2_POINT),
                  NavigationPoint(WAREHOUSE_ENTER_1_POINT), NavigationPoint(START_ENTER_POINT),
                  NavigationPoint(START_POINT))

    B_MODULE_5 = (NavigationPoint(START_POINT, Corrective(CorrectiveSensor.PING, 20)),
                  NavigationPoint(START_ENTER_POINT), NavigationPoint(WAREHOUSE_ENTER_1_POINT),
                  NavigationPoint(WAREHOUSE_ENTER_2_POINT), NavigationPoint(WAREHOUSE_ENTER_3_POINT),
                  NavigationPoint(WAREHOUSE_1_POINT))

    B_MODULE_6 = (NavigationPoint(ORCHARD_CORRIDOR_ENTER_1_POINT), NavigationPoint(ORCHARD_ENTER_POINT),
                  NavigationPoint(START_ENTER_POINT), NavigationPoint(START_POINT))

    B_MODULE_7 = (NavigationPoint(START_POINT, Corrective(CorrectiveSensor.PING, 20)),
                  NavigationPoint(START_ENTER_POINT), NavigationPoint(ORCHARD_ENTER_POINT),
                  NavigationPoint(ORCHARD_CORRIDOR_ENTER_1_POINT), NavigationPoint(ORCHARD_1_POINT))

    B_MODULE_11 = (NavigationPoint(START_POINT, Corrective(CorrectiveSensor.PING, 20)),
                   NavigationPoint(START_ENTER_POINT), NavigationPoint(WAREHOUSE_ENTER_1_POINT),
                   NavigationPoint(WAREHOUSE_ENTER_2_POINT), NavigationPoint(WAREHOUSE_ENTER_3_POINT),
                   NavigationPoint(WAREHOUSE_1_POINT),
                   NavigationPoint(WAREHOUSE_1_POINT, Corrective(CorrectiveSensor.PING, 30)))

    B_MODULE_12 = (NavigationPoint(ORCHARD_ENTER_POINT),
                   NavigationPoint(ORCHARD_ENTER_POINT, Corrective(CorrectiveSensor.PING, 30)),
                   NavigationPoint(WAREHOUSE_ENTER_1_POINT), NavigationPoint(WAREHOUSE_ENTER_2_POINT),
                   NavigationPoint(WAREHOUSE_ENTER_3_POINT), NavigationPoint(WAREHOUSE_1_POINT))

    ORCHARD_CORRIDOR_ENTER_1 = (NavigationPoint(ORCHARD_CORRIDOR_EXIT_1_POINT), )
