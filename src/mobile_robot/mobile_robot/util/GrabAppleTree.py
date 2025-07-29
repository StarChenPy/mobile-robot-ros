import time

import rclpy.node

from typing import List
from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.VisionService import VisionService


@singleton
class GrabAppleTree:
    def __init__(self, node: rclpy.node.Node, direction: Direction):
        self.logger = Logger()
        self.node = node

        self.direction = direction
        self.max_move_distance = 0.68
        self.max_grab_distance = 0.46
        self.min_grab_distance = 0.27

        self.basket_1 = []
        self.basket_2 = []
        self.basket_3 = []

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)

    def has_apple(self):
        apple_types = set(FruitType.apples())
        for i in range(1, 4):
            basket = getattr(self, f"basket_{i}")
            if basket and any(fruit in apple_types for fruit in basket):
                return True
        return False

    def find_fruits(self, fruit=None) -> List[IdentifyResult]:
        identify = self.vision.get_onnx_identify_depth()

        print(identify)

        result = []
        for i in identify:
            if fruit:
                if FruitType(i.class_id) not in fruit:
                    continue
            if i.distance > self.max_move_distance:
                continue
            result.append(i)
        return result

    def close_tree(self, distance: float):
        """靠近果树"""
        ArmMovement.motion(self.arm)
        self.move.rotate(90)
        if self.direction == Direction.LEFT:
            self.move.line(distance)
        if self.direction == Direction.RIGHT:
            self.move.line(-distance)
        self.move.rotate(-90)

    def grab_apple_from_tree(self):
        ArmMovement.identify_tree_fruit(self.arm, self.direction)
        fruits = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
        prev_move_len = 0

        if not fruits:
            self.logger.warn("没有检测到水果!")
            ArmMovement.motion(self.arm)
            return False

        # 靠近果树
        min_distance = min(self.find_fruits(), key=lambda fruit: fruit.distance).distance
        max_distance = max(self.find_fruits(), key=lambda fruit: fruit.distance).distance
        piff_dis = 0
        if max_distance > self.max_grab_distance:
            piff_dis = min_distance - self.min_grab_distance
            # piff_dis = max_distance - self.max_grab_distance
            self.close_tree(piff_dis)

        fruits.sort(key=lambda fruit: fruit.box.get_rectangle_center().x)
        for i in fruits:
            center = i.box.get_rectangle_center()
            fruit_type = FruitType(i.class_id)

            distance = 0.3
            if i.distance != -1:
                distance = i.distance + 0.04
                self.logger.info(f"抓取苹果使用深度信息 {distance}")
            else:
                self.logger.warn(f"深度相机没有深度信息，使用默认值 {distance}")

            if self.direction == Direction.LEFT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)
            elif self.direction == Direction.RIGHT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
            else:
                raise ValueError()
            move_distance = move_distance + 0.215

            self.arm.lift(0)
            ArmMovement.open_half_gripper(self.arm)
            self.move.line(move_distance - prev_move_len)
            prev_move_len = move_distance

            ArmMovement.grab_apple_on_tree(self.arm, self.direction, (distance - 0.34 - piff_dis) * 100, center.y > 220)
            for i in range(1, 4):
                basket = getattr(self, f"basket_{i}")
                if fruit_type in basket:
                    ArmMovement.put_fruit_to_basket(self.arm, i)
                    basket.remove(fruit_type)

        ArmMovement.motion(self.arm)
        return True
