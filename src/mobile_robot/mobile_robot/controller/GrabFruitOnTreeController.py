import rclpy.node

from ..param import ArmMovement as Movement
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Util
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

        Movement.grab_basket_to_warehouse(self.__arm, 1)
        Movement.grab_basket_to_warehouse(self.__arm, 2)
        Movement.grab_basket_to_warehouse(self.__arm, 3)

        # Movement.recognition_orchard_tree(self.__arm)
        #
        # result = self.__vision.get_onnx_identify_result(True)
        # grabber = FruitGrabber(self.__robot, self.__arm, self.__move)
        # grabber.run(result, {FruitType.GREEN_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.RED_APPLE: 3})

    # def move_to_grab(self):
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 10)))
    #     self.__move.line(0.37)
    #
    # def grab_fruit_on_tree_bottom_left(self):
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 10)), is_block=False)
    #     self.__move.rotate(90)
    #     self.__move.line(0.05)
    #     self.__arm.control(ArmMovement(MotorMovement(-83, 3), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(MotorMovement(-83, 32), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(-20, 0, 15, 10)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(-20, 0, 15, 6.5)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(-20, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))
    #
    #     Movement.put_fruit_into_basket(self.__arm, 1)
    #
    # def grab_fruit_on_tree_top_left(self):
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 10)), is_block=False)
    #     self.__move.rotate(90)
    #     self.__move.line(0.05)
    #     self.__arm.control(ArmMovement(MotorMovement(-95, 3), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(MotorMovement(-95, 26), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 15, 14)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 15, 6.5)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))
    #
    #     Movement.put_fruit_into_basket(self.__arm, 1)
    #
    #
    # def grab_fruit_on_tree_bottom_right(self):
    #     self.__arm.control(ArmMovement(MotorMovement(90, 3), ServoMotor(0, 0, 0, 10)), is_block=False)
    #     self.__move.rotate(-90)
    #     self.__move.line(0.05)
    #     self.__arm.control(ArmMovement(MotorMovement(81, 3), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(MotorMovement(81, 32), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(40, 0, 15, 10)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(40, 0, 15, 6.5)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(20, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(90, 3), ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))
    #
    #     Movement.put_fruit_into_basket(self.__arm, 1)
    #
    # def grab_fruit_on_tree_top_right(self):
    #     self.__arm.control(ArmMovement(MotorMovement(90, 3), ServoMotor(0, 0, 0, 10)), is_block=False)
    #     self.__move.rotate(-90)
    #     self.__move.line(0.05)
    #     self.__arm.control(ArmMovement(MotorMovement(95, 3), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(MotorMovement(95, 26), ServoMotor(0, 0, 0, 10)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 15, 14)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 15, 6.5)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(90, 3), ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))
    #
    #     Movement.put_fruit_into_basket(self.__arm, 1)
    #
    #
    # def grab_fruit_on_tree_bottom_center(self):
    #     self.__move.line(-0.1)
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 14)), is_block=False)
    #     self.__move.rotate(89)
    #     self.__move.line(-0.12)
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 14)))
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 32), ServoMotor(-175, 0, 0, 14)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(-175, 0, 15, 14)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(-175, 0, 15, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(-110, 32), ServoMotor(-175, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(-110, 3), ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))
    #
    #     Movement.put_fruit_into_basket(self.__arm, 1)
    #
    # def grab_fruit_on_tree_top_center(self):
    #     self.__move.line(-0.1)
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 14)), is_block=False)
    #     self.__move.rotate(90)
    #     self.__move.line(-0.12)
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 14)))
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 26), ServoMotor(0, 0, 0, 14)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 8, 14)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 8, 6.5)))
    #     self.__arm.control(ArmMovement(servo=ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(-90, 3), ServoMotor(0, 0, 0, 6.5)))
    #     self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))
    #
    #     Movement.put_fruit_into_basket(self.__arm, 1)


    def identify_and_grab(self):
        """
        抓树上水果
        """
        Movement.recognition_orchard_tree(self.__arm)
        while True:
            result = self.__vision.get_onnx_identify_result()
            if not result:
                break
            result = self.__vision.get_onnx_identify_result()
            if not result:
                break

            fruit = FruitType.get_by_value(result[0].classId)

            d = {FruitType.GREEN_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.RED_APPLE: 3}
            location_on_tree = Util.get_fruit_location_on_tree(self.__vision, fruit)

            Movement.grab_fruit_on_tree(self.__arm, self.__move, location_on_tree)
            Movement.put_fruit_into_basket(self.__arm, d[fruit])

            Movement.recognition_orchard_tree(self.__arm)

        self.__arm.control(Movement.MOVING)
