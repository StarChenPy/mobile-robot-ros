from ..popo.Corrective import Corrective
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..popo.CorrectivePoint import CorrectivePoint

# 矫正点
START_CORRECTIVE_POINT = CorrectivePoint(1.07, -2.89, 180, [Corrective(Direction.BACK, 0.422)])
ORCHARD_ENTER_CORRECTIVE_POINT = CorrectivePoint(0.42, -3.65, 0, [Corrective(Direction.BACK, 0.422), Corrective(Direction.RIGHT, 0.135)])
ORCHARD_CORRIDOR_ENTER_1_CORRECTIVE_POINT = CorrectivePoint(1.91, -3.65, 90, [Corrective(Direction.BACK, 0.135 + 0.222)])
ORCHARD_CORRIDOR_EXIT_1_CORRECTIVE_POINT = CorrectivePoint(1.91, -0.42, 0, [Corrective(Direction.LEFT, 0.2), Corrective(Direction.BACK, 0.21 + 0.222)])

START_POINT = NavigationPoint(1.1, -2.9, 180)
START_ENTER_POINT = NavigationPoint(0.42, -2.9, 0)

# 果仓坐标
WAREHOUSE_ENTER_1_POINT = NavigationPoint(0.42, -2.0, 0)
WAREHOUSE_ENTER_2_POINT = NavigationPoint(1.1, -2.0, 0)
WAREHOUSE_ENTER_3_POINT = NavigationPoint(1.1, -1.12, 0)

WAREHOUSE_1_POINT = NavigationPoint(0.56, -1.12, 180)
WAREHOUSE_2_POINT = NavigationPoint(0.56, -0.72, 180)
WAREHOUSE_3_POINT = NavigationPoint(0.56, -0.32, 180)

# 果园坐标
ORCHARD_ENTER_POINT = NavigationPoint(0.5, -3.65, 0)

ORCHARD_ENTER_POINT_1 = NavigationPoint(1.2, -3.65, 0)

ORCHARD_CORRIDOR_ENTER_1_POINT = NavigationPoint(1.93, -3.65, 90)
ORCHARD_CORRIDOR_EXIT_1_POINT = NavigationPoint(1.93, -0.42, -90)

ORCHARD_CORRIDOR_ENTER_2_POINT = NavigationPoint(2.73, -3.65, 90)
ORCHARD_CORRIDOR_EXIT_2_POINT = NavigationPoint(2.73, -0.4, -90)

# ORCHARD_CORRIDOR_ENTER_3_POINT = NavigationPoint(3.6, -3.65, 90)
# ORCHARD_CORRIDOR_EXIT_3_POINT = NavigationPoint(3.6, -0.4, -90)

# 果园1\2\3\4\5号点
ORCHARD_1_POINT = NavigationPoint(1.91, -2.85, 90)
ORCHARD_2_POINT = NavigationPoint(1.91, -2.35, 90)
ORCHARD_3_POINT = NavigationPoint(1.91, -1.85, 90)
ORCHARD_4_POINT = NavigationPoint(2.73, -2.9, 90)
ORCHARD_5_POINT = NavigationPoint(2.73, -2.4, 90)
