import time

import rclpy.node

from ..param import ArmMovement
from ..service.ArmService import ArmService
from ..service.VisionService import VisionService


class TestModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)

    def run(self):
        self.__arm.control(ArmMovement.TEST, is_block=False)
        self.__arm.control(ArmMovement.TEST, is_block=False)
