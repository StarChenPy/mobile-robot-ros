import time

import rclpy

from ..popo.ArmMovement import ArmMovement
from ..service.ArmService import ArmService
from ..util.Singleton import singleton


@singleton
class ArmController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__arm = ArmService(node)

    def control(self, movement: ArmMovement, speed=45.0, is_block=False):
        self.__arm.control(movement, speed, is_block)
        time.sleep(0.5)

    def reset(self):
        self.__arm.back_origin()
