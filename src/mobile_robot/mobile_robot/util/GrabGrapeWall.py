import math
import time

import rclpy

from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement
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
    def __init__(self, node: rclpy.node.Node, direction: Direction):
        self.logger = Logger()
        self.node = node

        self.direction = direction
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
        center = grape.box.get_rectangle_center()

        distance = 0.32
        x_distance = 0
        if self.direction == Direction.LEFT:
            ir = self.sensor.get_ir_left()
            if 0.2 < ir < 0.35:
                distance = ir
            x_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)
        elif self.direction == Direction.RIGHT:
            ir = self.sensor.get_ir_right()
            if 0.2 < ir < 0.35:
                distance = ir
            x_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
        y_distance = Math.pixel_to_distance_from_bottom(center.y, distance)
        self.move.line(x_distance)

        # distance 要减去夹爪5cm, x_distance要加上从夹爪到旋转中心的34cm
        angle = Math.calculate_right_triangle_angle(distance - 0.05, 0.34)

        lift_height = 32 - (y_distance * 100)
        self.arm.lift(lift_height, is_block=False)
        self.arm.nod_servo(90)

        if self.direction == Direction.LEFT:
            self.arm.rotary_servo(-(90 - angle))
            self.arm.rotate(180 - angle)
        if self.direction == Direction.RIGHT:
            self.arm.rotary_servo(90 - angle)
            self.arm.rotate(-180 + angle)

        self.arm.nod_servo(50)
        self.arm.wait_finish()
        time.sleep(0.2)
        ArmMovement.close_gripper_grape(self.arm)
        self.arm.telescopic_servo(0)

    def find_grape_and_grab(self, target_point: NavigationPoint):
        ArmMovement.identify_grape(self.arm, self.direction)

        start_yaw = Math.round_right_angle(self.sensor.get_odom_data().w)
        end_yaw = target_point.yaw

        target_point.yaw = start_yaw

        self.move.navigation([target_point], 0.15, False, False)
        while self.move.get_status():
            # 如果框子里没有要抓的水果了，直接返回
            if not self.has_grape():
                break

            odom = self.sensor.get_odom_data()
            d = math.sqrt(math.pow((target_point.x - odom.x), 2) + math.pow((target_point.y - odom.y), 2))
            # 走0.5m 固定矫正一次
            if d % 0.5 < 0.1:
                # 矫正角度
                angle_from_wall = self.sensor.get_angle_from_wall(self.direction.invert())
                self.logger.info(f"行走了一定距离, 进行角度矫正")
                if abs(angle_from_wall) < 10:
                    self.sensor.init_odom_yaw(start_yaw - angle_from_wall)
                    self.logger.info(f"矫正当前角度为 {start_yaw - angle_from_wall} 度.")
                else:
                    self.logger.warn("角度偏差过大，不予置信.")

            grape = self.vision.find_fruit(self.basket_1 + self.basket_2 + self.basket_3)
            if not grape:
                continue

            # 检测到了，就停下来再看一遍
            self.move.stop_navigation()
            grape = self.vision.find_fruit(self.basket_1 + self.basket_2 + self.basket_3)
            if grape:
                fruit_type = FruitType(grape.class_id)

                for i in range(1, 4):
                    basket = getattr(self, f"basket_{i}")
                    if fruit_type in basket:
                        basket.remove(fruit_type)
                        self.grab_grape(grape)
                        ArmMovement.put_fruit_to_basket(self.arm, i)
                        break

                ArmMovement.identify_grape(self.arm, self.direction)

            # 矫正角度
            angle_from_wall = self.sensor.get_angle_from_wall(self.direction.invert())
            if abs(angle_from_wall) < 10:
                self.sensor.init_odom_yaw(start_yaw - angle_from_wall)

            # 继续导航
            self.move.navigation([target_point], 0.15, False, False)

        ArmMovement.motion(self.arm)
        target_point.yaw = end_yaw
        self.move.navigation([target_point])
