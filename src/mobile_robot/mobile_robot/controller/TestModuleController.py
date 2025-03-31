import time

import rclpy.node

from ..param import ArmMovement
from ..popo.Direction import Direction
from ..service.ArmService import ArmService
from ..service.VisionService import VisionService


class TestModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)

    def run(self):
        ArmMovement.recognition_orchard(self.__arm, Direction.RIGHT)
        time.sleep(2)
        result = self.__vision.get_onnx_identify_result()
        for p in result:
            point = p.box.get_rectangle_center()
            depth = self.__vision.get_depth_data(point)
            print(depth)
            self.__arm.grab_fruit(depth, Direction.RIGHT)
            break

        self.__arm.control(ArmMovement.MOVING)
