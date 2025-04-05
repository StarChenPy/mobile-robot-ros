import rclpy.node

from ..popo.Direction import Direction
from ..popo.FruitHeight import FruitHeight
from ..service.ArmService import ArmService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.Logger import Logger


class TestModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)
        self.__sensor = SensorService(node)

    def run(self):
        while True:
            input("等待...")

            left_dis = self.__sensor.get_distance_from_wall(Direction.LEFT)
            right_dis = self.__sensor.get_distance_from_wall(Direction.RIGHT)

            print(f"距离左墙 {left_dis}，距离右墙 {right_dis}")
