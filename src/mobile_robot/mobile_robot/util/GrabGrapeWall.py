import math
import time

import rclpy

from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService


@singleton
class GrabGrapeWall:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()
        self.node = node

        self.direction = None
        self.speed = 0.1
        self.basket_1 = []
        self.basket_2 = []
        self.basket_3 = []

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)
        self.sensor = SensorService(node)

    def has_grape(self):
        grape_types = set(FruitType.grapes())
        for i in range(1, 4):
            basket = getattr(self, f"basket_{i}")
            if basket and any(fruit in grape_types for fruit in basket):
                return True
        return False

    def grab_grape(self, grape: IdentifyResult):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓葡萄!")
            return

        if not grape:
            self.logger.error("识别结果为空!")
            return

        center = grape.box.get_rectangle_center()

        distance = 0.30

        x_distance = 0.04 if self.direction == Direction.LEFT else 0.02
        if self.direction == Direction.LEFT:
            x_distance += Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)
        elif self.direction == Direction.RIGHT:
            x_distance += Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
        y_distance = Math.pixel_to_distance_from_bottom(center.y, distance)
        self.move.line(x_distance)

        # distance要加上夹爪的5cm, x_distance要加上从夹爪到旋转中心的34cm
        angle = Math.calculate_right_triangle_angle(distance - 0.1, 0.34)

        lift_height = 34 - (y_distance * 100)
        self.arm.lift(lift_height, is_block=False)
        self.arm.nod_servo(90)

        ArmMovement.open_grape_gripper(self.arm)
        if self.direction == Direction.LEFT:
            self.arm.rotary_servo(-(90 - angle))
            self.arm.rotate(180 - angle)
        if self.direction == Direction.RIGHT:
            self.arm.rotary_servo((90 - angle))
            self.arm.rotate(-180 + angle)

        self.arm.nod_servo(60)
        self.arm.wait_finish()
        time.sleep(0.3)
        ArmMovement.close_gripper_grape(self.arm)
        time.sleep(0.3)

        if self.direction == Direction.LEFT:
            self.arm.rotate(180)
        elif self.direction == Direction.RIGHT:
            self.arm.rotate(-180)

    def find_grape_and_grab(self, waypoint_name: str):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓葡萄!")
            return

        ArmMovement.identify_grape(self.arm, self.direction)

        self.move.my_navigation(waypoint_name, 0.1, False)
        while self.move.get_my_status():
            grape = self.vision.find_fruit(self.basket_1 + self.basket_2 + self.basket_3, True)
            if not grape:
                continue

            # 检测到了，就停下来再看一遍
            self.move.stop_my_navigation()
            time.sleep(0.5)
            self.move.rotation_correction(Direction.FRONT, True, 10)
            time.sleep(0.5)
            grape = self.vision.find_fruit(self.basket_1 + self.basket_2 + self.basket_3, True)
            if grape:
                self.logger.info(f"抓取到葡萄: {grape.class_id}")
                fruit_type = FruitType(grape.class_id)

                for i in range(1, 4):
                    basket = getattr(self, f"basket_{i}")
                    if fruit_type in basket:
                        basket.remove(fruit_type)
                        self.grab_grape(grape)
                        ArmMovement.put_fruit_to_basket(self.arm, i, True)
                        break

                # 如果框子里没有要抓的水果了，直接返回
                if not self.has_grape():
                    break

                self.move.rotation_correction(Direction.FRONT, True, 10)
                ArmMovement.identify_grape(self.arm, self.direction)

        # 如果框子里没有要抓的水果了，直接返回
            if not self.has_grape():
                self.logger.info("框子里没有要抓的葡萄了，停止抓取。")
                break

            # 继续导航
            self.move.my_navigation(waypoint_name, 0.1, False)

        ArmMovement.motion(self.arm)
