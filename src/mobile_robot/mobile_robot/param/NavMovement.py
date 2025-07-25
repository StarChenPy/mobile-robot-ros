from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..service.MoveService import MoveService


START_POINT = CorrectivePoint(0.35, 0.4, 90, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.35)])

CORRECTIVE_POINT_1 = CorrectivePoint(0.5, 3.5, 0, [Corrective(Direction.BACK, 0.5), Corrective(Direction.LEFT, 0.5)])

CORRECTIVE_POINT_2 = CorrectivePoint(3.65, 3.5, -90, [Corrective(Direction.BACK, 0.5), Corrective(Direction.LEFT, 0.35)])

YELLOW_STATION_1 = NavigationPoint(0.89, 3.7, 0)
RED_STATION_1 = NavigationPoint(0.89, 3.7, 0)

YELLOW_STATION_2 = NavigationPoint(2.89, 3.7, 0)
RED_STATION_2 = NavigationPoint(2.89, 3.7, 0)

YELLOW_STATION_3 = NavigationPoint(3.7, 2.89, -90)
RED_STATION_3 = NavigationPoint(3.7, 2.89, -90)

IDENTIFY_TREE_1_1 = NavigationPoint(0.91, 2.35, 0)
IDENTIFY_TREE_1_2 = NavigationPoint(1.82, 3.25, -90)

VINEYARD_1 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_2 = CorrectivePoint(3.22, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_3 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_4 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_5 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_6 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_7 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])
VINEYARD_8 = CorrectivePoint(3.6, 1.675, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.325)])

def start_to_yellow_station_1(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, YELLOW_STATION_1])

def start_to_yellow_station_2(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, YELLOW_STATION_2])

def yellow_station_2_to_red_station_1(move: MoveService):
    move.navigation([RED_STATION_1])

def yellow_station_1_to_vineyard(move: MoveService):
    move.navigation([YELLOW_STATION_3, VINEYARD_1])

def vineyard_1_to_yellow_station_3(move: MoveService):
    move.navigation([VINEYARD_1, RED_STATION_3])

def start_to_tree1(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, IDENTIFY_TREE_1_1])

def start_to_vineyard_1(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, CORRECTIVE_POINT_2, VINEYARD_1])
