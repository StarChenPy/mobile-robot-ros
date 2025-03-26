from ..popo.Corrective import Corrective
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..popo.CorrectivePoint import CorrectivePoint

# 矫正点
START_CORRECTIVE_POINT = CorrectivePoint(1.15, -2.83, -90, [Corrective(Direction.BACK, 0.33)])
WAREHOUSE_ENTER_0_CORRECTIVE_POINT = CorrectivePoint(0.42, -3.65, 0, [Corrective(Direction.BACK, 0.422), Corrective(Direction.RIGHT, 0.357)])
ORCHARD_CORRIDOR_ENTER_1_CORRECTIVE_POINT = CorrectivePoint(2, -3.45, 90, [Corrective(Direction.BACK, 0.555), Corrective(Direction.LEFT, 0.49)])
ORCHARD_CORRIDOR_START_1_CORRECTIVE_POINT = CorrectivePoint(2, -0.42, 0, [Corrective(Direction.BACK, 0.49), Corrective(Direction.LEFT, 0.418)])
WAREHOUSE_CORRECTIVE_POINT = CorrectivePoint(1.09, -0.36, 180, [Corrective(Direction.BACK, 0.40), Corrective(Direction.RIGHT, 0.36)])

START_POINT = NavigationPoint(1.15, -2.83, -90)
START_ENTER_POINT = NavigationPoint(1.15, -3.65, None)

# 果仓坐标
WAREHOUSE_ENTER_0_POINT = NavigationPoint(0.42, -3.65, None)
WAREHOUSE_ENTER_1_POINT = NavigationPoint(0.42, -2.0, None)
WAREHOUSE_ENTER_2_POINT = NavigationPoint(1.1, -2.0, None)
WAREHOUSE_ENTER_3_POINT = NavigationPoint(1.1, -1.12, None)

WAREHOUSE_1_POINT = NavigationPoint(0.53, -1.09, 180)
WAREHOUSE_2_POINT = NavigationPoint(0.53, -0.38, 180)
WAREHOUSE_3_POINT = NavigationPoint(0.53, -0.38, 180)

# 果园坐标
ORCHARD_CORRIDOR_ENTER_1_POINT = NavigationPoint(2, -3.65, 90)
ORCHARD_CORRIDOR_START_1_POINT = NavigationPoint(2, -3.45, 90)
ORCHARD_CORRIDOR_END_1_POINT = NavigationPoint(2, -0.72, -90)
ORCHARD_CORRIDOR_EXIT_1_POINT = NavigationPoint(2, -0.42, -90)

ORCHARD_CORRIDOR_ENTER_2_POINT = NavigationPoint(2.82, -3.65, 90)
ORCHARD_CORRIDOR_START_2_POINT = NavigationPoint(2.82, -3.45, 90)
ORCHARD_CORRIDOR_END_2_POINT = NavigationPoint(2.82, -0.72, -90)
ORCHARD_CORRIDOR_EXIT_2_POINT = NavigationPoint(2.82, -0.36, -90)

# ORCHARD_CORRIDOR_ENTER_3_POINT = NavigationPoint(3.6, -3.65, 90)
# ORCHARD_CORRIDOR_EXIT_3_POINT = NavigationPoint(3.6, -0.4, -90)

# 果园1\2\3\4\5号点
ORCHARD_1_POINT = NavigationPoint(2, -2.59, 90)
ORCHARD_2_POINT = NavigationPoint(2, -1.84, 90)
ORCHARD_3_POINT = NavigationPoint(2, -1.05, 90)
ORCHARD_4_POINT = NavigationPoint(2.82, -2.59, 90)
ORCHARD_5_POINT = NavigationPoint(2.82, -1.82, 90)
