import rclpy

from ..param.arm_movement import ArmMovementParam
from ..service.ArmService import ArmService
from ..util.Singleton import singleton


@singleton
class ArmController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__arm = ArmService(node)

    def control(self, movement: ArmMovementParam, is_block=False):
        self.__arm.control(movement, is_block)

