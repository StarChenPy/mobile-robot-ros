import rclpy

from ..model.ArmMovement import ArmMovement
from ..model.FruitType import FruitType
from ..model.MotorMovement import MotorMovement
from ..model.ServoMotor import ServoMotor
from ..param import ArmMovement as Movement
from ..param import ShandongTrialsNavigationPath
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Util
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class ShandongTrialsController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)
        self.__sensor = SensorService(node)
        self.__robot = RobotService(node)
        self.__move = MoveService(node)

    def run(self):
        self.__robot.with_robot_connect()
        self.__arm.back_origin()
        self.__arm.control(Movement.MOVING)
        self.__robot.with_start_button()

        self.__move.navigation(ShandongTrialsNavigationPath.START_TO_TREE_1)
        self.identify_and_grab()
        self.__move.navigation(ShandongTrialsNavigationPath.TREE_1_TO_TREE_2)
        self.identify_and_grab()
        self.__move.navigation(ShandongTrialsNavigationPath.TREE_2_TO_TREE_2)
        self.identify_and_grab()
        self.__move.navigation(ShandongTrialsNavigationPath.TREE_2_TO_TREE_3)
        self.__move.navigation(ShandongTrialsNavigationPath.TREE_3_TO_TREE_4)
        self.__move.navigation(ShandongTrialsNavigationPath.TREE_4_TO_TREE_START)

    def identify_and_grab(self):
        self.__arm.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, 0, 0, 15)))
        self.__arm.control(ArmMovement(MotorMovement(180, 22), ServoMotor(-170, 0, 0, 15)))
        while True:
            result = self.__vision.get_onnx_identify_result()
            if not result:
                break
            result = self.__vision.get_onnx_identify_result()
            if not result:
                break

            fruit = FruitType.get_by_value(result[0].classId)

            d = {FruitType.GREEN_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.RED_APPLE: 3}
            location_on_tree = Util.get_fruit_location_on_tree(self.__vision, fruit)

            Movement.grab_fruit_on_tree(self.__arm, self.__move, location_on_tree)
            Movement.put_fruit_into_basket(self.__arm, d[fruit])

            self.__arm.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, 0, 0, 15)))
            self.__arm.control(ArmMovement(MotorMovement(180, 22), ServoMotor(-170, 0, 0, 15)))

        self.__arm.control(Movement.MOVING)
