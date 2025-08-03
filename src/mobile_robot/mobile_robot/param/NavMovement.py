from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint


START_POINT = CorrectivePoint(0.35, 0.4, 90, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.35)])

CORRECTIVE_POINT_1 = CorrectivePoint(0.4, 3.7, 0, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.3)])
CORRECTIVE_POINT_2 = CorrectivePoint(3.7, 3.6, -90, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.3)])

YELLOW_STATION_1 = NavigationPoint(1.0, 3.7, 0)
RED_STATION_1 = NavigationPoint(1.0, 3.7, 0)
YELLOW_STATION_2 = NavigationPoint(3.0, 3.7, 0)
RED_STATION_2 = NavigationPoint(3.0, 3.7, 0)
YELLOW_STATION_3 = NavigationPoint(3.7, 3.0, -90)
RED_STATION_3 = NavigationPoint(3.7, 3.0, -90)

TREE_1 = NavigationPoint(2.9, 3.0, None)
TREE_2 = NavigationPoint(2.03, 2.5, None)
TREE_3 = NavigationPoint(1.05, 2.6, None)
TREE_4 = NavigationPoint(1.6, 3.5, None)

POINT_A = NavigationPoint(0.4, 3.05, None)
POINT_B = NavigationPoint(1.46, 3.07, None)
POINT_D = NavigationPoint(2.19, 3.07, 0)
POINT_F = NavigationPoint(1.56, 2.55, None)

VINEYARD_1 = CorrectivePoint(3.6, 1.65, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.32)])
VINEYARD_2 = NavigationPoint(2.0, 1.65, 180)
VINEYARD_3 = CorrectivePoint(1.31, 1.65, 0, [Corrective(Direction.BACK, 0.6), Corrective(Direction.LEFT, 0.32)])
VINEYARD_4_0 = NavigationPoint(2.0, 0.99, 0)
VINEYARD_4_180 = NavigationPoint(2.0, 0.99, 180)
VINEYARD_5 = CorrectivePoint(1.31, 0.99, 0, [Corrective(Direction.BACK, 0.6), Corrective(Direction.RIGHT, 0.32)])
VINEYARD_6 = NavigationPoint(3.4, 0.99, 0)
VINEYARD_7 = CorrectivePoint(3.55, 0.33, 180, [Corrective(Direction.BACK, 0.45), Corrective(Direction.LEFT, 0.33)])
VINEYARD_8 = NavigationPoint(1.31, 0.33, 180)
