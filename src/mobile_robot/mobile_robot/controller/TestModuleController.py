import rclpy.node

from ..popo.ArmMovement import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitHeight import FruitHeight
from ..popo.ServoMotor import ServoMotor
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
        # self.__arm.back_origin()
        # self.__arm.grab_fruit(FruitHeight.TALL.value, Direction.RIGHT)

        while True:
            input("等待...")

            left_dis = self.__sensor.get_distance_from_wall(Direction.LEFT)
            right_dis = self.__sensor.get_distance_from_wall(Direction.RIGHT)

            print(f"雷达距离左墙 {left_dis}，距离右墙 {right_dis}")

            left_dis = self.__sensor.get_ir_left()
            right_dis = self.__sensor.get_ir_right()

            print(f"红外距离左墙 {left_dis}，距离右墙 {right_dis}")
