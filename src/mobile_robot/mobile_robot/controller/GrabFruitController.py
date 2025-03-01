import time

import rclpy

from ..param.ArmMovement import ArmMovementParam
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.NavigationService import NavigationService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.Singleton import singleton


def get_fruit_height(height: float) -> FruitHeight:
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

    def vision(self) -> list[IdentifyResult]:
        return self.__vision.get_onnx_identify_result()

    def patrol_the_line(self, target_point: NavigationPoint, target_fruit: list[FruitType], is_other_side=False) -> FruitType | None:
        """
        给予路径点与水果类型, 在前往路径点的过程中寻找指定水果
        @param target_point: 目标路径点
        @param target_fruit: 指定水果列表
        @param is_other_side: 水果是否在另一边
        @return 抓的水果
        """
        self.__ready_to_identify(is_other_side)
        self.__navigation.navigation([target_point], 0.05, False)

        while rclpy.ok() and self.__navigation.get_status():
            results = [result for result in self.__vision.get_onnx_identify_result() if 20 < result.box.get_rectangle_center().x < 400]

            if not results:
                continue

            result = results[0]
            if result.fruit_type in target_fruit:
                self.__navigation.stop_navigation()
                self.execute_grab_sequence(is_other_side)
                return result.fruit_type

        return None

    def __ready_to_identify(self, is_other_side: bool):
        """准备机械臂到视觉识别姿态"""
        if is_other_side:
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_LEFT, 20)
        else:
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT, 20)

    def execute_grab_sequence(self, is_other_side: bool):
        """执行抓取动作序列"""
        result = self.__vision.get_onnx_identify_result()[0]
        fruit_center = result.box.get_rectangle_center()
        depth = self.__vision.get_depth_data(fruit_center)
        angular_offset = (fruit_center.x - 320) / 10

        print("深度数据为:", depth)

        self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_RIGHT, 20)

        match get_fruit_height(fruit_center.y):
            case FruitHeight.TALL:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_TALL_LEFT if is_other_side else ArmMovementParam.READY_GRAB_APPLE_TALL_RIGHT
                movement = ArmMovementParam.GRAB_APPLE_TALL_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_TALL_RIGHT
                ready_movement.value.servo.telescopic = depth - 13
            case FruitHeight.MIDDLE:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_MIDDLE_LEFT if is_other_side else ArmMovementParam.READY_GRAB_APPLE_MIDDLE_RIGHT
                movement = ArmMovementParam.GRAB_APPLE_MIDDLE_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_MIDDLE_RIGHT
                ready_movement.value.servo.telescopic = depth - 8
            case FruitHeight.LOW:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_LOW_LEFT if is_other_side else ArmMovementParam.READY_GRAB_APPLE_LOW_RIGHT
                movement = ArmMovementParam.GRAB_APPLE_LOW_LEFT if is_other_side else ArmMovementParam.GRAB_APPLE_LOW_RIGHT
                ready_movement.value.servo.telescopic = depth - 8
            case _:
                ready_movement = ArmMovementParam.MOVING
                movement = ArmMovementParam.MOVING

        ready_movement.value.motor.rotate += angular_offset
        self.__arm.control(ready_movement, 20, True)
        time.sleep(1)
        self.__arm.control(movement, 20, True)
        time.sleep(1)
        self.__arm.control(ArmMovementParam.MOVING, 20)
