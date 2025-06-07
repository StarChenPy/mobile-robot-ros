import rclpy.node

from ..param import ArmMovement as Movement
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Util
from ..util.FruitGrabber import FruitGrabber
from ..util.Logger import Logger
from ..util.Singleton import singleton


# 车距离树75cm的地方拍照
# 拍完照往前开20cm


@singleton
class GrabFruitOnTreeController:
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

        grabber = FruitGrabber(self.__robot, self.__arm, self.__move, self.__sensor, self.__vision)
        grabber.run({FruitType.GREEN_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.RED_APPLE: 3})

    def identify_and_grab(self):
        """
        抓树上水果
        """
        Movement.recognition_orchard_tree(self.__arm)
        while True:
            result = self.__vision.get_onnx_identify_depth()
            if not result:
                break
            result = self.__vision.get_onnx_identify_depth()
            if not result:
                break

            fruit = FruitType.get_by_value(result[0].classId)

            d = {FruitType.GREEN_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.RED_APPLE: 3}
            location_on_tree = Util.get_fruit_location_on_tree(self.__vision, fruit)

            Movement.grab_fruit_on_tree(self.__arm, self.__move, location_on_tree)
            Movement.put_fruit_into_basket(self.__arm, d[fruit])

            Movement.recognition_orchard_tree(self.__arm)

        self.__arm.control(Movement.MOVING)
