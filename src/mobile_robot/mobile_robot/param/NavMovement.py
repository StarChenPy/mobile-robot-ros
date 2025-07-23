from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..service.MoveService import MoveService


START_POINT = CorrectivePoint(0.3, 0.4, 90, [Corrective(Direction.BACK, 0.4), Corrective(Direction.LEFT, 0.33)])


def start_to_yellow_station_2(move: MoveService):
    move.navigation([START_POINT, NavigationPoint(0.33, 2.6, 90), NavigationPoint(3.7, 2.6, 0), NavigationPoint(3.72, 3.7, 90)])

def yellow_station_2_to_red_station_1(move: MoveService):
    move.navigation([NavigationPoint(3.65, 1.76, 0)])

def yellow_station_2_to_vineyard(move: MoveService):
    move.navigation([NavigationPoint(3.65, 1.76, 0)])