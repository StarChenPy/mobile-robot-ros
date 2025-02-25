import copy

import rclpy

from ..param.arm_movement import ArmMovementParam
from ..param.navigation_path import NavPath
from ..popo.FruitHeight import FruitHeight
from ..popo.Rectangle import Rectangle
from ..robot.util import math
from ..service.ArmService import ArmService
from ..service.NavigationService import NavigationService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.Singleton import singleton


@singleton
class GrabFruitController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__arm = ArmService(node)
        self.__navigation = NavigationService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def vision(self):
        return self.__vision.get_identify_result()

    def grab_fruits(self, nav_path: NavPath, task: dict[int, list[str]], direction="left" or "right") -> bool:
        """
        扫描并抓取水果，主控制流程，带篮子
        """
        self.__prepare_for_recognition(direction)
        self.__navigation.navigation(nav_path, 0.05, False)

        while rclpy.ok() and self.__navigation.get_status():
            results = self.__get_valid_detections()

            for result in results:
                if self.__process_single_fruit(result, task):
                    self.__arm.control(ArmMovementParam.MOVING, 20, True)
                    return True

        self.__arm.control(ArmMovementParam.MOVING, 20, True)
        return False

    def get_fruit_height(self, box: Rectangle) -> FruitHeight:
        height = box.get_rectangle_center().y

        if height < 150:
            return FruitHeight.TALL
        elif height < 280:
            return FruitHeight.MIDDLE
        else:
            return FruitHeight.LOW

    # region 辅助方法
    def __prepare_for_recognition(self, direction="left" or "right"):
        """准备视觉识别状态"""
        if direction == "left":
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_LEFT, 20)
        elif direction == "right":
            self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT, 20)

    def __get_valid_detections(self) -> list:
        """获取有效检测结果并进行初步过滤"""
        return [
            result for result in self.__vision.get_identify_result()
            # x的范围是0～460
            if 120 < result.box.get_rectangle_center().x < 400
        ]

    def __process_single_fruit(self, result, task: dict) -> bool:
        """处理单个水果的抓取流程，返回是否需要终止检测循环"""
        # 篮子选择逻辑
        basket_id = 0
        for bid, target_classes in task.items():
            if result.classId in target_classes:
                task[bid].remove(result.classId)
                basket_id = bid
                break

        if basket_id != 0:
            self.__navigation.stop()
            self.__execute_grab_sequence(result.box)
            return True

        return False

    def __execute_grab_sequence(self, box):
        """执行抓取动作序列"""
        self.__arm.control(ArmMovementParam.READY_GRAB_APPLE, 20)
        match self.get_fruit_height(box):
            case FruitHeight.TALL:
                distance = self.__get_fruit_distance(ArmMovementParam.READY_GRAB_APPLE)
                movement = ArmMovementParam.GRAB_APPLE_TALL
                movement.value.servo.telescopic = distance - 11
                self.__arm.control(movement, 20)
            case FruitHeight.MIDDLE:
                distance = self.__get_fruit_distance(ArmMovementParam.READY_GRAB_APPLE)
                movement = ArmMovementParam.GRAB_APPLE_MIDDLE
                movement.value.servo.telescopic = distance - 6
                self.__arm.control(movement, 20)
            case FruitHeight.LOW:
                distance = self.__get_fruit_distance(ArmMovementParam.READY_GRAB_APPLE)
                movement = ArmMovementParam.GRAB_APPLE_LOW
                movement.value.servo.telescopic = distance - 6
                self.__arm.control(movement, 20)

    def __execute_placement_sequence(self, basket_id: int) -> bool:
        """执行放置动作序列，返回是否需要终止检测循环"""
        self.__arm.control(ArmMovementParam.READY_PUT_FRUIT_INTO_BASKET, 20, True)

        match basket_id:
            case 1:
                self.__arm.control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_1, 20)
            case 2:
                self.__arm.control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_2, 20)
            case 3:
                self.__arm.control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_3, 20)
            case 4:
                return True  # 特殊终止信号

        return False

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
        ir_claws_left = math.calculate_adjacent_side(ir_claws_left, angle)

        # 扫描中间
        self.__arm.control(center_movement, True)
        print("扫描中间", center_movement.value.motor.rotate)
        ir_claws_center = self.__sensor.get_ir_claws()

        # 扫描右侧
        right_movement.value.motor.rotate -= angle
        self.__arm.control(right_movement, True)
        print("扫描右侧", right_movement.value.motor.rotate)
        ir_claws_right = self.__sensor.get_ir_claws()
        ir_claws_right = math.calculate_adjacent_side(ir_claws_right, angle)

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
