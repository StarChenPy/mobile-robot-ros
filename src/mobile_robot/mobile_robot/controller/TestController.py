import rclpy

from ..param import ArmMovement, NavMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.Logger import Logger
from ..util.Singleton import singleton
from ..util.StationEnum import Station


@singleton
class TestController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.node = node
        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

    def run(self):
        self.robot.with_robot_connect()
        # self.arm.back_origin()

        # self.vision.show_photo(self.vision.photograph())


        # self.sensor.init_odom_all(NavigationPoint(0, 0, 0))
        # self.move.navigation([NavigationPoint(2, 0, 0)], 0.1)

        # ArmMovement.identify_grape(self.arm, Direction.RIGHT)
        # ArmMovement.grab_grape_on_wall(self.arm, Direction.RIGHT, True)

        # self.arm.back_origin()
        # tree = GrabAppleTree(self.node, Direction.LEFT)
        # tree.basket_1 = [FruitType.RED_APPLE]
        # tree.basket_2 = [FruitType.GREEN_APPLE]
        # tree.grab_apple_from_tree()
        # ArmMovement.motion(self.arm)

        while True:
            input("Press Enter to continue...")
            print("左墙: ", self.sensor.get_angle_from_wall(Direction.LEFT))
            print("右墙: ", self.sensor.get_angle_from_wall(Direction.RIGHT))
