import enum
import time

import rclpy.node

from ..param import ArmMovement as Movement
from ..popo.ArmMovement import ArmMovement
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.MotorMovement import MotorMovement
from ..popo.NavigationPoint import NavigationPoint
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.Logger import Logger


# 车距离树75cm的地方拍照
# 拍完照往前开20cm

class FruitLocationOnTree(enum.Enum):
    TOP_LEFT = 0
    BOTTOM_LEFT = 1
    TOP_CENTER = 2
    BOTTOM_CENTER = 3
    TOP_RIGHT = 4
    BOTTOM_RIGHT = 5


class TestModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)
        self.__sensor = SensorService(node)
        self.__robot = RobotService(node)
        self.__move = MoveService(node)

    def get_fruit_location_on_tree(self, fruit: FruitType) -> FruitLocationOnTree:
        result = self.__vision.get_onnx_identify_result()

        while 0 in [i.distance for i in result]:
            result = self.__vision.get_onnx_identify_result()

        for i in result:
            if i.classId != fruit.value:
                continue

            if i.distance < 0.35:
                if i.box.get_rectangle_center().y < 240:
                    return FruitLocationOnTree.TOP_CENTER
                else:
                    return FruitLocationOnTree.BOTTOM_CENTER
            else:
                if i.box.get_rectangle_center().x < 300:
                    if i.box.get_rectangle_center().y < 240:
                        return FruitLocationOnTree.TOP_LEFT
                    else:
                        return FruitLocationOnTree.BOTTOM_LEFT
                elif i.box.get_rectangle_center().x > 340:
                    if i.box.get_rectangle_center().y < 240:
                        return FruitLocationOnTree.TOP_RIGHT
                    else:
                        return FruitLocationOnTree.BOTTOM_RIGHT
                else:
                    self.__logger.warn(f"奇怪，为什么会有 {i.classId} {i.box.get_rectangle_center()} 的苹果？")
                    return FruitLocationOnTree.TOP_CENTER

    def run(self):
        self.__robot.with_robot_connect()
        self.__arm.back_origin()

        self.get_fruit_on_tree(15.5)

        self.__move.navigation([CorrectivePoint(0, 0, 0, [Corrective(Direction.BACK, 0.37)]), NavigationPoint(0, -0.73, None), NavigationPoint(0.73, -0.75, 90)])

        self.get_fruit_on_tree(10)


    def get_fruit_on_tree(self, distance: float):
        self.__arm.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, 0, 0, 15)))
        self.__arm.control(ArmMovement(MotorMovement(180, 28), ServoMotor(0, 0, 0, 15)))
        while True:
            result = self.__vision.get_onnx_identify_result()

            if not result:
                break

            fruit = FruitType.get_by_value(result[0].classId)

            d = {FruitType.GREEN_APPLE: 1, FruitType.YELLOW_APPLE: 2, FruitType.RED_APPLE: 3}
            location_on_tree = self.get_fruit_location_on_tree(fruit)

            print(location_on_tree)

            self.grub_fruit_for_tree(location_on_tree)
            Movement.put_fruit_into_basket(self.__arm, d[fruit])

            self.__arm.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, 0, 0, 15)))
            self.__arm.control(ArmMovement(MotorMovement(180, 28), ServoMotor(0, 0, 0, 15)))
            self.__sensor.ping_revise(distance, True)

        self.__arm.control(Movement.MOVING)

    def grub_fruit_for_tree(self, location_on_tree: FruitLocationOnTree):
        CONFIGS = {
            FruitLocationOnTree.TOP_LEFT:    (196, 18, 20, 0.35, 12),
            FruitLocationOnTree.TOP_CENTER:  (180, 18, 0, 0.2, 2),
            FruitLocationOnTree.TOP_RIGHT:   (164, 18, -20, 0.35, 12),
            FruitLocationOnTree.BOTTOM_LEFT: (203, 30, 20, 0.33, 12),
            FruitLocationOnTree.BOTTOM_CENTER:(180, 30, 0, 0.2, 7),
            FruitLocationOnTree.BOTTOM_RIGHT: (157, 30, -20, 0.33, 12)
        }

        match location_on_tree:
            case loc if loc in CONFIGS:
                arm_pos, arm_lift, rotate, line, telescopic = CONFIGS[loc]
            case _:
                raise RuntimeError("不支持的水果位置")

        # 左上水果
        # 准备抓
        self.__arm.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(0, 0, 0, 15)))
        self.__move.rotate(rotate)
        self.__move.line(line)
        self.__arm.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(0, 0, telescopic, 15)))
        # 抓取
        self.__arm.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(0, 0, telescopic, 6.5)))
        time.sleep(1)
        # 抓离
        self.__arm.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(0, 0, 0, 6.5)))
        self.__move.line(-line)
        self.__move.rotate(-rotate)
