import time

import rclpy

from ..popo.ArmMovement import ArmMovement
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class GrabFruitOnGroundController:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)
        self.__sensor = SensorService(node)
        self.__robot = RobotService(node)
        self.__move = MoveService(node)

    def run(self):
        # self.__arm.back_origin()

        self.__arm.control(ArmMovement(MotorMovement(180, 20), ServoMotor(0, -90, 15, 15)))

        while True:
            q = input("等待...")
            if q == "q":
                break

            for e in self.__vision.get_onnx_identify_depth():
                center = e.box.get_rectangle_center()
                print(e.classId, center, e.distance)

                point = Math.pixel_to_world(center, 0.24)
                triangle_angle = -Math.calculate_right_triangle_angle(point.x, point.y + 0.285)
                hypotenuse = Math.calculate_hypotenuse(point.x, point.y) - 0.12

                if abs(triangle_angle) > 2:
                    self.__move.rotate(triangle_angle)
                if hypotenuse > 0:
                    self.__move.line(hypotenuse)

                self.grab_fruit_on_tree()

                if hypotenuse > 0:
                    self.__move.line(-hypotenuse)
                if abs(triangle_angle) > 2:
                    self.__move.rotate(-triangle_angle)

    def grab_fruit_on_tree(self):
        # 准备抓
        self.__arm.control(ArmMovement(MotorMovement(180, 20), ServoMotor(0, -90, 15, 15)))
        self.__arm.control(ArmMovement(MotorMovement(180, 33), ServoMotor(0, -90, 15, 15)))
        self.__arm.control(ArmMovement(MotorMovement(180, 33), ServoMotor(0, -90, 15, 6.5)))
        time.sleep(1)
        self.__arm.control(ArmMovement(MotorMovement(180, 20), ServoMotor(0, -90, 15, 6.5)))
