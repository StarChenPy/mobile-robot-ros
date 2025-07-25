import time

import rclpy

from ..param import ArmMovement, NavMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TestController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.node = node
        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)


    def find_fruit(self, fruit=None):
        identify = self.vision.get_onnx_identify_depth()
        for i in identify:
            if fruit:
                if FruitType(i.class_id) not in fruit:
                    continue
            return i
        return None

    def grab_apple_from_tree(self, direction: Direction):
        # ArmMovement.identify_tree_fruit(self.arm, Direction.LEFT)
        input("等待...")
        fruit = self.find_fruit([FruitType.RED_APPLE])
        if fruit is not None:
            center = fruit.box.get_rectangle_center()

            distance = 0.3
            if fruit.distance != 0:
                distance = fruit.distance

            x_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance) - 0.215
            if direction == Direction.LEFT:
                x_distance = -x_distance
            elif direction != Direction.RIGHT:
                raise ValueError("不支持的方向!")

            self.move.line(x_distance)

            ArmMovement.open_half_gripper(self.arm)
            self.arm.telescopic_servo(distance)
            ArmMovement.close_gripper_apple(self.arm)
            self.arm.telescopic_servo(0)

            ArmMovement.put_fruit_to_basket(self.arm, 1)

            self.move.line(-x_distance)

    def run(self):
        self.robot.with_robot_connect()
        # self.arm.back_origin()

        # self.sensor.init_odom_all(NavigationPoint(0, 0, 0))
        # self.move.navigation([NavigationPoint(1, 0, 0)])

        # ArmMovement.identify_grape(self.arm, Direction.RIGHT)
        # ArmMovement.grab_grape_on_wall(self.arm, Direction.RIGHT, True)
        # self.arm.back_origin()
        self.grab_apple_from_tree(Direction.LEFT)


        # while True:
        #     input("Press Enter to continue...")
        #     self.move.corrective(NavMovement.START_POINT)

