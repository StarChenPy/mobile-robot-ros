import time

import rclpy

from ..param.ArmMovement import ArmMovementParam
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..popo.Rectangle import Rectangle
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
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
        self.__navigation = MoveService(node)
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
                self.__navigation.stop_navigation()
                self.execute_grab_sequence(result.box, is_other_side)
                print(3)
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
            if 20 < result.box.get_rectangle_center().x < 400
        ]

    def execute_grab_sequence(self, box: Rectangle, is_other_side: bool):
        """执行抓取动作序列"""
        self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_RIGHT, 20)

        match get_fruit_height(box):
            case FruitHeight.TALL:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_TALL_LEFT if is_other_side else ArmMovementParam.READY_GRAB_APPLE_TALL_RIGHT
                movement = ArmMovementParam.GRAB_APPLE_TALL_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_TALL_RIGHT
            case FruitHeight.MIDDLE:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_MIDDLE_LEFT if is_other_side else ArmMovementParam.READY_GRAB_APPLE_MIDDLE_RIGHT
                movement = ArmMovementParam.GRAB_APPLE_MIDDLE_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_MIDDLE_RIGHT
            case FruitHeight.LOW:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_LOW_LEFT if is_other_side else ArmMovementParam.READY_GRAB_APPLE_LOW_RIGHT
                movement = ArmMovementParam.GRAB_APPLE_LOW_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_LOW_RIGHT
            case _:
                ready_movement = ArmMovementParam.MOVING
                movement = ArmMovementParam.MOVING

        self.__arm.control(ready_movement, 20, True)
        time.sleep(1)
        self.__arm.control(movement, 20, True)
        time.sleep(1)
        self.__arm.control(ArmMovementParam.MOVING, 20)
