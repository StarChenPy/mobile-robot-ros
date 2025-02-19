import enum

from ..util.data_type import Pose, NavigationPoint, CorrectivePoint, SensorType

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
B_MODULE_1_POINT = NavigationPoint(1.05, 0, 0)

# 矫正点
START_CORRECTIVE_POINT = CorrectivePoint(1.1, -2.9, SensorType.PING, 180, 30, 90, 20)
ORCHARD_ENTER_CORRECTIVE_POINT = CorrectivePoint(0.5, -3.7, SensorType.PING, 0, 20, 90, 10)
WAREHOUSE_CORRECTIVE_POINT = CorrectivePoint(1.15, -0.4, SensorType.PING, -180, 10, -90, 10)
ORCHARD_CORRIDOR_EXIT_1_CORRECTIVE_POINT = CorrectivePoint(2, -0.4, SensorType.PING, -90, 10, -180, 10)

START_POINT = NavigationPoint(1.1, -2.9, -180)
START_ENTER_POINT = NavigationPoint(0.5, -2.9, -180)

# 果仓坐标
WAREHOUSE_ENTER_1_POINT = NavigationPoint(0.5, -2, 0)
WAREHOUSE_ENTER_2_POINT = NavigationPoint(1.15, -2, 0)
WAREHOUSE_ENTER_3_POINT = NavigationPoint(1.15, -1.04, 0)
WAREHOUSE_1_POINT = NavigationPoint(0.62, -1.04, 90)
WAREHOUSE_2_POINT = NavigationPoint(0.62, -0.8, 90)
WAREHOUSE_3_POINT = NavigationPoint(0.62, -0.4, 90)

# 果园坐标
ORCHARD_ENTER_POINT = NavigationPoint(0.5, -3.7, 0)

ORCHARD_CORRIDOR_ENTER_1_POINT = NavigationPoint(2, -3.7, 90)
ORCHARD_CORRIDOR_EXIT_1_POINT = NavigationPoint(2, -0.4, -90)

ORCHARD_CORRIDOR_ENTER_2_POINT = NavigationPoint(2.8, -3.7, 90)
ORCHARD_CORRIDOR_EXIT_2_POINT = NavigationPoint(2.8, -0.4, -90)

ORCHARD_CORRIDOR_ENTER_3_POINT = NavigationPoint(3.6, -3.7, 90)
ORCHARD_CORRIDOR_EXIT_3_POINT = NavigationPoint(3.6, -0.4, -90)

# 果园1号点
ORCHARD_1_POINT = Pose(2, -2.92, 90)


class NavPath(enum.Enum):
    # STARTING_POINT2PICKING_POINT = (STARTING_POINT, TURNING_POINT, PICKING_ENTRANCE)
    #
    # PICKING_CORRIDOR_1 = (PICKING_CORRIDOR_1_START, PICKING_CORRIDOR_1_END)
    B_MODULE_1 = (B_MODULE_1_POINT, )

    B_MODULE_4 = (WAREHOUSE_1_POINT,
                  WAREHOUSE_CORRECTIVE_POINT,
                  WAREHOUSE_ENTER_3_POINT,
                  WAREHOUSE_ENTER_2_POINT,
                  WAREHOUSE_ENTER_1_POINT,
                  START_ENTER_POINT,
                  START_POINT)

    B_MODULE_5 = (START_CORRECTIVE_POINT,
                  START_ENTER_POINT,
                  WAREHOUSE_ENTER_1_POINT,
                  WAREHOUSE_ENTER_2_POINT,
                  WAREHOUSE_ENTER_3_POINT,
                  WAREHOUSE_1_POINT)

    B_MODULE_6 = (ORCHARD_CORRIDOR_ENTER_1_POINT,
                  ORCHARD_ENTER_POINT,
                  START_ENTER_POINT,
                  START_POINT)

    START_TO_ORCHARD_ENTER_1 = (START_CORRECTIVE_POINT,
                                START_ENTER_POINT,
                                ORCHARD_ENTER_POINT,
                                ORCHARD_CORRIDOR_ENTER_1_POINT,
                                ORCHARD_CORRIDOR_ENTER_1_POINT, CorrectivePoint(SensorType.PING, 7))

    B_MODULE_7 = (START_CORRECTIVE_POINT,
                  START_ENTER_POINT,
                  ORCHARD_ENTER_POINT,
                  ORCHARD_CORRIDOR_ENTER_1_POINT,
                  ORCHARD_1_POINT)

    B_MODULE_11 = (START_CORRECTIVE_POINT,
                   START_ENTER_POINT,
                   WAREHOUSE_ENTER_1_POINT,
                   WAREHOUSE_ENTER_2_POINT,
                   WAREHOUSE_ENTER_3_POINT,
                   WAREHOUSE_1_POINT)

    B_MODULE_12 = (ORCHARD_ENTER_POINT,
                   ORCHARD_ENTER_POINT, CorrectivePoint(SensorType.PING, 30),
                   WAREHOUSE_ENTER_1_POINT,
                   WAREHOUSE_ENTER_2_POINT,
                   WAREHOUSE_ENTER_3_POINT,
                   WAREHOUSE_1_POINT)

    ORCHARD_CORRIDOR_ENTER_1 = (ORCHARD_CORRIDOR_EXIT_1_POINT, )

    ORCHARD_CORRIDOR_ENTER_2 = (ORCHARD_CORRIDOR_EXIT_2_POINT,
                                ORCHARD_CORRIDOR_ENTER_2_POINT)

    EXIT_1_POINT_TO_WAREHOUSE_1_POINT = (ORCHARD_CORRIDOR_EXIT_1_POINT, CorrectivePoint(SensorType.PING, 30),
                                         ORCHARD_CORRIDOR_ENTER_1_POINT,
                                         ORCHARD_ENTER_POINT,
                                         WAREHOUSE_ENTER_1_POINT,
                                         WAREHOUSE_ENTER_2_POINT,
                                         WAREHOUSE_ENTER_3_POINT,
                                         WAREHOUSE_1_POINT)

    WAREHOUSE_TO_ORCHARD_ENTER_1 = (WAREHOUSE_CORRECTIVE_POINT,
                                    WAREHOUSE_3_POINT,
                                    WAREHOUSE_2_POINT,
                                    WAREHOUSE_1_POINT,
                                    ORCHARD_ENTER_POINT,
                                    ORCHARD_CORRIDOR_ENTER_1_POINT)

    WAREHOUSE_1_TO_WAREHOUSE_2 = (WAREHOUSE_2_POINT, )

    WAREHOUSE_1_TO_WAREHOUSE_3 = (WAREHOUSE_3_POINT, )

    ENTER_2_POINT_TO_WAREHOUSE_1_POINT = (ORCHARD_CORRIDOR_ENTER_2_POINT, CorrectivePoint(SensorType.PING, 7),
                                          ORCHARD_ENTER_POINT,
                                          WAREHOUSE_ENTER_1_POINT,
                                          WAREHOUSE_ENTER_2_POINT,
                                          WAREHOUSE_ENTER_3_POINT,
                                          WAREHOUSE_1_POINT)

    EXIT_1_TO_EXIT_2 = (ORCHARD_CORRIDOR_EXIT_1_CORRECTIVE_POINT,
                        ORCHARD_CORRIDOR_EXIT_2_POINT)

    EXIT_2_TO_ENTER_2 = (ORCHARD_CORRIDOR_ENTER_2_POINT, )
