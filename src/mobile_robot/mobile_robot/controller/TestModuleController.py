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

        while True:
            s = input("等待...")
            if s == "q":
                break
            for e in self.__vision.get_onnx_identify_result():
                print(e)

        self.__arm.control(ArmMovement.MOVING)
