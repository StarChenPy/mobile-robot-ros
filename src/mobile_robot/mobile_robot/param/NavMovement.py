from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..service.MoveService import MoveService


START_POINT = CorrectivePoint(0.35, 0.4, 90, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.35)])

CORRECTIVE_POINT_1 = CorrectivePoint(0.4, 3.7, 0, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.3)])

CORRECTIVE_POINT_2 = CorrectivePoint(3.7, 3.6, -90, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.3)])

YELLOW_STATION_1 = NavigationPoint(0.89, 3.7, 0)
RED_STATION_1 = NavigationPoint(0.89, 3.7, 0)

YELLOW_STATION_2 = NavigationPoint(2.89, 3.7, 0)
RED_STATION_2 = NavigationPoint(2.89, 3.7, 0)

YELLOW_STATION_3 = NavigationPoint(3.7, 3.11, -90)
RED_STATION_3 = NavigationPoint(3.7, 3.11, -90)

IDENTIFY_TREE_1_1 = NavigationPoint(1.26, 2.65, -90)
IDENTIFY_TREE_1_2 = NavigationPoint(1.5, 2.93, 0)
IDENTIFY_RANDOM_TREE_1_1 = NavigationPoint(1.14, 2.65, -90)
IDENTIFY_RANDOM_TREE_1_2 = NavigationPoint(1.5, 2.93, 0)

VINEYARD_1 = CorrectivePoint(3.6, 1.65, 180, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.32)])
VINEYARD_2 = NavigationPoint(2.1, 1.65, 180)
VINEYARD_3 = CorrectivePoint(1.31, 1.65, 0, [Corrective(Direction.BACK, 0.6), Corrective(Direction.LEFT, 0.32)])
VINEYARD_4_0 = NavigationPoint(2.1, 0.99, 0)
VINEYARD_4_180 = NavigationPoint(2.1, 0.99, 180)
VINEYARD_5 = CorrectivePoint(1.31, 0.99, 0, [Corrective(Direction.BACK, 0.6), Corrective(Direction.RIGHT, 0.32)])
VINEYARD_6 = NavigationPoint(3.4, 0.99, 0)
VINEYARD_7 = CorrectivePoint(3.55, 0.33, 180, [Corrective(Direction.BACK, 0.45), Corrective(Direction.LEFT, 0.33)])
VINEYARD_8 = CorrectivePoint(1.31, 0.33, 0, [Corrective(Direction.BACK, 0.6), Corrective(Direction.RIGHT, 0.33)])

def start_to_yellow_station_1(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, YELLOW_STATION_1])

def start_to_yellow_station_2(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, YELLOW_STATION_2])

def yellow_station_2_to_red_station_1(move: MoveService):
    move.navigation([CORRECTIVE_POINT_1, RED_STATION_1])

def yellow_station_1_to_vineyard(move: MoveService):
    move.navigation([YELLOW_STATION_3, VINEYARD_1])

def vineyard_1_to_yellow_station_3(move: MoveService):
    move.navigation([VINEYARD_1, RED_STATION_3])

def start_to_tree1(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, IDENTIFY_TREE_1_1])

def start_to_vineyard_1(move: MoveService):
    move.navigation([START_POINT, CORRECTIVE_POINT_1, CORRECTIVE_POINT_2, VINEYARD_1])
