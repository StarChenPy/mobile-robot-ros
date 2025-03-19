import time

import rclpy

from ..param.ArmMovement import ArmMovementParam
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..popo.Rectangle import Rectangle
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.Math import Math
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
        self.__move = MoveService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def vision(self):
        return self.__vision.get_onnx_identify_result()

    def patrol_the_line(self, target_point: NavigationPoint, target_fruit: FruitType, is_other_side=False) -> bool:
        self.__logger.info(f"[GrabFruitController] 准备开始 目标点 {target_point}, 目标水果 {target_fruit}")
        self.__ready_to_identify(is_other_side)

        odom = self.__sensor.get_odom_data()
        points = Math.point_to_point(NavigationPoint(odom.x, odom.y, odom.w), target_point, 0.4)

        for point in points:
            print(self.__sensor.get_odom_data())
            self.__move.navigation([point], 0.2, True)

            # results = self.__vision.get_onnx_identify_result()
            results = []

            self.__logger.info(f"[GrabFruitController] 识别到的内容: {results}")

            result = None
            for r in results:
                if r.box.get_area() < 5000:
                    print(r.box.get_area())
                    continue
                if r.classId != target_fruit.value:
                    continue
                result = r

            if not result:
                continue

            center = result.box.get_rectangle_center()
            travel_distance = (320 - center.x) / 2000
            self.__move.line(-travel_distance if is_other_side else travel_distance)
            self.execute_grab_sequence(get_fruit_height(result.box), is_other_side)
            return True

        return False

    def __ready_to_identify(self, is_other_side: bool):
        """准备机械臂到视觉识别姿态"""
        if is_other_side:
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_LEFT, 45)
        else:
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT, 45)

    def execute_grab_sequence(self, height: FruitHeight, is_other_side: bool):
        """执行抓取动作序列"""
        if is_other_side:
            ArmMovementParam.READY_GRAB_APPLE.value.motor.rotate = -ArmMovementParam.READY_GRAB_APPLE.value.motor.rotate
        self.__arm.control(ArmMovementParam.READY_GRAB_APPLE, 45)

        match height:
            case FruitHeight.TALL:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_TALL
                movement = ArmMovementParam.GRAB_APPLE_TALL
            case FruitHeight.MIDDLE:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_MIDDLE
                movement = ArmMovementParam.GRAB_APPLE_MIDDLE
            case FruitHeight.LOW:
                ready_movement = ArmMovementParam.READY_GRAB_APPLE_LOW
                movement = ArmMovementParam.GRAB_APPLE_LOW
            case _:
                ready_movement = ArmMovementParam.MOVING
                movement = ArmMovementParam.MOVING

        if is_other_side:
            ready_movement.value.motor.rotate = -ready_movement.value.motor.rotate
            movement.value.motor.rotate = -movement.value.motor.rotate

        self.__arm.control(ready_movement, 45, True)
        time.sleep(1)
        self.__arm.control(movement, 45, True)
        time.sleep(1)
        self.__arm.control(ArmMovementParam.MOVING, 45)
