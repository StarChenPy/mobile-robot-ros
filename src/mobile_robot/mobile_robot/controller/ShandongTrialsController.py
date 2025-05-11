import time

import rclpy

from ..param import ArmMovement as Movement
from ..param import ShandongTrialsNavigationPath
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.FruitGrabber import FruitGrabber
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

        grabber = FruitGrabber(self.__robot, self.__arm, self.__move)
        task = {FruitType.RED_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.GREEN_APPLE: 3}

        self.__move.navigation(ShandongTrialsNavigationPath.START_TO_TREE_1)
        result = self.__vision.get_onnx_identify_result(True)
        grabber.run(result, task)

        self.__move.navigation(ShandongTrialsNavigationPath.TREE_1_TO_TREE_2)
        result = self.__vision.get_onnx_identify_result(True)
        grabber.run(result, task)

        self.__move.navigation(ShandongTrialsNavigationPath.TREE_2_TO_TREE_2)
        result = self.__vision.get_onnx_identify_result(True)
        grabber.run(result, task)

        self.__move.navigation(ShandongTrialsNavigationPath.TREE_2_TO_TREE_3)
        result = self.__vision.get_onnx_identify_result(True)
        grabber.run(result, task)

        self.__move.navigation(ShandongTrialsNavigationPath.TREE_3_TO_TREE_4)
        result = self.__vision.get_onnx_identify_result(True)
        grabber.run(result, task)

        self.__move.navigation(ShandongTrialsNavigationPath.TREE_4_TO_TREE_START)
