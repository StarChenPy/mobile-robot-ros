from ..param import NavMovement, ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.GrabAppleTree import GrabAppleTree
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.Logger import Logger
from ..util.Singleton import singleton
from ..util.StationEnum import Station


@singleton
class TaskEController:
    def __init__(self, node):
        self.node = node
        self.logger = Logger()

        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

    def run(self):
        self.robot.with_robot_connect()
        self.arm.back_origin()
        ArmMovement.motion(self.arm)
        self.robot.set_start_led(False)

        self.robot.with_start_button()

        self.grab_baskets()
        self.grab_apples()
        self.grab_grapes()
        self.put_baskets()

        self.move.navigation([NavMovement.START_POINT])
        self.robot.set_start_led(False)

    def grab_baskets(self):
        self.move.navigation([NavMovement.START_POINT, NavMovement.CORRECTIVE_POINT_1, NavMovement.YELLOW_STATION_1])
        Station.YELLOW_STATION_1.revise(self.node)
        self.arm.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        self.move.navigation([NavMovement.YELLOW_STATION_2])
        Station.YELLOW_STATION_2.revise(self.node)
        self.arm.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        self.move.navigation([NavMovement.CORRECTIVE_POINT_2, NavMovement.YELLOW_STATION_3])
        Station.YELLOW_STATION_3.revise(self.node)
        self.arm.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 3)
        ArmMovement.motion(self.arm)

    def grab_apples(self):
        grab_apple_tree = GrabAppleTree(self.node, Direction.LEFT)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE] * 6
        grab_apple_tree.basket_2 = [FruitType.GREEN_APPLE] * 6
        grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE] * 6

        self.move.navigation([NavMovement.IDENTIFY_TREE_1_1])
        grab_apple_tree.grab_apple_from_tree()

        if grab_apple_tree.has_apple():
            self.move.navigation([NavMovement.IDENTIFY_TREE_1_2])
            grab_apple_tree.grab_apple_from_tree()

    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node, Direction.RIGHT)
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE] * 6
        grab_grape_wall.basket_2 = [FruitType.GREEN_GRAPE] * 6
        grab_grape_wall.basket_3 = [FruitType.YELLOW_GRAPE] * 6

        # 抓第一个走廊
        self.move.navigation([NavMovement.CORRECTIVE_POINT_2, NavMovement.VINEYARD_1])
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_3)

        # 抓第二个走廊的一半
        self.move.navigation([NavMovement.VINEYARD_2, NavMovement.VINEYARD_4_180])
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_5)

        # 抓第二个走廊的另一半
        self.move.navigation([NavMovement.VINEYARD_4_0])
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_6)

        # 抓第三个走廊
        self.move.navigation([NavMovement.VINEYARD_7])
        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_8)

        # 回到第一个走廊
        self.move.navigation([NavMovement.VINEYARD_7, NavMovement.VINEYARD_6, NavMovement.VINEYARD_4_180, NavMovement.VINEYARD_2, NavMovement.VINEYARD_1])


    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node, Direction.RIGHT)
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE] * 6
        grab_grape_wall.basket_2 = [FruitType.GREEN_GRAPE] * 6
        grab_grape_wall.basket_3 = [FruitType.YELLOW_GRAPE] * 6

        # 抓第一个走廊
        self.move.navigation([NavMovement.VINEYARD_1])
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_3)

        # 抓第二个走廊的一半
        self.move.navigation([NavMovement.VINEYARD_2, NavMovement.VINEYARD_4_180])
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_5)

        # 抓第二个走廊的另一半
        self.move.navigation([NavMovement.VINEYARD_4_0])
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_6)

        # 抓第三个走廊
        self.move.navigation([NavMovement.VINEYARD_7])
        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.find_grape_and_grab(NavMovement.VINEYARD_8)

        # 回到第一个走廊
        self.move.navigation([NavMovement.VINEYARD_7, NavMovement.VINEYARD_6, NavMovement.VINEYARD_4_180, NavMovement.VINEYARD_2, NavMovement.VINEYARD_1])


    def put_baskets(self):
        # 去放第1个框子
        self.move.navigation([NavMovement.RED_STATION_3])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

        # 去放第2个框子
        self.move.navigation([NavMovement.RED_STATION_2])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 2)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

        # 去放第3个框子
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.RED_STATION_1])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 3)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)
