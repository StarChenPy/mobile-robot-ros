import copy

import rclpy

from ..param.ArmMovement import ArmMovementParam
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..popo.Rectangle import Rectangle
from ..service.ArmService import ArmService
from ..service.NavigationService import NavigationService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.Singleton import singleton


def get_fruit_height(box: Rectangle) -> FruitHeight:
    height = box.get_rectangle_center().y

    if height < 150:
        return FruitHeight.TALL
    elif height < 280:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.LOW


@singleton
class GrabFruitController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__arm = ArmService(node)
        self.__navigation = NavigationService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def vision(self):
        return self.__vision.get_onnx_identify_result()

    def patrol_the_line(self, target_point: NavigationPoint, target_fruit: FruitType, is_other_side=False) -> bool:
        self.__ready_to_identify(is_other_side)
        self.__navigation.navigation([target_point], 0.05, False)

        while rclpy.ok() and self.__navigation.get_status():
            results = self.__get_valid_detections()

            if not results:
                continue

            result = results[0]
            if result.classId == target_fruit.value:
                self.__navigation.stop()
                self.__execute_grab_sequence(result.box, is_other_side)
                return True

        return False

    def __ready_to_identify(self, is_other_side: bool):
        """准备机械臂到视觉识别姿态"""
        if is_other_side:
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_LEFT, 20)
        else:
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT, 20)

    def __get_valid_detections(self) -> list[IdentifyResult]:
        """获取有效检测结果并进行初步过滤"""
        return [
            result for result in self.__vision.get_onnx_identify_result()
            # x的范围是0～480
            if 120 < result.box.get_rectangle_center().x < 400
        ]

    def __execute_grab_sequence(self, box, is_other_side: bool):
        """执行抓取动作序列"""
        self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_RIGHT, 20)

        distance = self.__get_fruit_distance(ArmMovementParam.READY_GRAB_APPLE_RIGHT)
        print(distance)

        match get_fruit_height(box):
            case FruitHeight.TALL:
                movement = ArmMovementParam.GRAB_APPLE_TALL_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_TALL_RIGHT
                movement.value.servo.telescopic = distance - 11
            case FruitHeight.MIDDLE:
                movement = ArmMovementParam.GRAB_APPLE_MIDDLE_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_MIDDLE_RIGHT
                movement.value.servo.telescopic = distance - 6
            case FruitHeight.LOW:
                movement = ArmMovementParam.GRAB_APPLE_LOW_RIGHT if is_other_side else ArmMovementParam.GRAB_APPLE_LOW_LEFT
                movement.value.servo.telescopic = distance - 6
            case _:
                movement = ArmMovementParam.MOVING

        self.__arm.control(movement, 20)
        self.__arm.control(ArmMovementParam.MOVING, 20)

    def __get_fruit_distance(self, movement: ArmMovementParam):
        angle = 15

        left_movement = copy.deepcopy(movement)
        center_movement = copy.deepcopy(movement)
        right_movement = copy.deepcopy(movement)

        print(id(left_movement), id(right_movement), id(center_movement))

        # 扫描左侧
        left_movement.value.motor.rotate += angle
        self.__arm.control(left_movement, True)
        print("扫描左侧", left_movement.value.motor.rotate)
        ir_claws_left = self.__sensor.get_ir_claws()
        ir_claws_left = Math.calculate_adjacent_side(ir_claws_left, angle)

        # 扫描中间
        self.__arm.control(center_movement, True)
        print("扫描中间", center_movement.value.motor.rotate)
        ir_claws_center = self.__sensor.get_ir_claws()

        # 扫描右侧
        right_movement.value.motor.rotate -= angle
        self.__arm.control(right_movement, True)
        print("扫描右侧", right_movement.value.motor.rotate)
        ir_claws_right = self.__sensor.get_ir_claws()
        ir_claws_right = Math.calculate_adjacent_side(ir_claws_right, angle)

        min_val = min(ir_claws_left, ir_claws_center, ir_claws_right)

        # 判断最小值来源并执行对应操作
        if ir_claws_left == min_val:
            # 左侧最小，执行左侧功能
            return (ir_claws_center + ir_claws_right) / 2
        elif ir_claws_center == min_val:
            # 中间最小，执行中间功能
            return (ir_claws_left + ir_claws_right) / 2
        else:
            # 右侧最小，执行右侧功能
            return (ir_claws_left + ir_claws_center) / 2
