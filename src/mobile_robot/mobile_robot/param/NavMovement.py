from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint


# 矫正点
C_START = CorrectivePoint(1.53, 2.5, 90, [Corrective(Direction.BACK, 0.3), Corrective(Direction.RIGHT, 0.3)])

C_1 = CorrectivePoint(0.3, 3.7, -90, [Corrective(Direction.BACK, 0.3), Corrective(Direction.RIGHT, 0.3)])
C_2 = CorrectivePoint(0.3, 1.0, 0, [Corrective(Direction.BACK, 0.3), Corrective(Direction.RIGHT, 0.38)])
C_3 = CorrectivePoint(0.92, 2.78, 90, [Corrective(Direction.FRONT, 0.6), Corrective(Direction.RIGHT, 0.3)])
C_4 = CorrectivePoint(1.52, 1.8, -90, [Corrective(Direction.BACK, 0.38), Corrective(Direction.LEFT, 0.31)])
C_5 = CorrectivePoint(0.6, 0.3, 180, [Corrective(Direction.FRONT, 0.6), Corrective(Direction.LEFT, 0.3)])
C_6 = CorrectivePoint(3.4, 0.3, 0, [Corrective(Direction.FRONT, 0.6), Corrective(Direction.RIGHT, 0.3)])
C_7 = CorrectivePoint(3.7, 0.92, 90, [Corrective(Direction.BACK, 0.3), Corrective(Direction.RIGHT, 0.3)])
C_8 = CorrectivePoint(3.7, 3.4, -90, [Corrective(Direction.BACK, 0.6), Corrective(Direction.LEFT, 0.3)])

# 站台
S_Y_1_L = NavigationPoint(0.3, 1.89, 90)
S_Y_1_R = NavigationPoint(0.3, 2.11, -90)
S_Y_2_R = NavigationPoint(1.52, 3.7, 180)
S_Y_3_L = NavigationPoint(1.61, 0.3, 180)
S_Y_3_R = NavigationPoint(1.39, 0.3, 0)

S_R_1_L = NavigationPoint(2.48, 3.7, 0)
S_R_1_R = NavigationPoint(2.7, 3.7, 180)
S_R_2_R = NavigationPoint(3.7, 1.59, 90)
S_R_3_L = NavigationPoint(2.51, 0.3, 180)
S_R_3_R = NavigationPoint(2.29, 0.3, 0)

# 树
T_1 = NavigationPoint(3, 1.48, None)
T_1_G = NavigationPoint(2.89, 0.93, 0)
T_2 = NavigationPoint(2.48, 2.08, None)
T_3 = NavigationPoint(3.4, 2.08, None)

# 葡萄园点
V_1 = NavigationPoint(0.93, 1, 90)
V_2 = NavigationPoint(1.52, 1.2, -90)
V_3 = NavigationPoint(1.82, 0.3, 180)
V_4 = NavigationPoint(2.2, 0.3, 0)

# 中间导航点
P_1 = NavigationPoint(1.53, 3.7, None)
P_2 = NavigationPoint(2.73, 3.7, None)
P_3 = NavigationPoint(2.73, 2.91, -90)
P_4 = NavigationPoint(2.42, 0.92, None)
P_5 = NavigationPoint(1.52, 0.3, None)
P_6 = NavigationPoint(0.93, 1.87, None)
P_7 = NavigationPoint(2.92, 2.08, None)
P_8 = NavigationPoint(2.43, 1.48, None)
