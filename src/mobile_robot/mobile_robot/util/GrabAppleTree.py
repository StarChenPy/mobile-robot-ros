import math

import rclpy.node

from typing import List
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
class GrabAppleTree:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()
        self.node = node

        self.direction = None
        self.max_grab_distance = 0.52
        self.min_grab_distance = 0.27

        self.basket_1 = []
        self.basket_2 = []
        self.basket_3 = []

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)
        self.sensor = SensorService(node)

    def has_apple(self):
        apple_types = set(FruitType.apples())
        for i in range(1, 4):
            basket = getattr(self, f"basket_{i}")
            if basket and any(fruit in apple_types for fruit in basket):
                return True
        return False

    def find_fruits(self, fruit=None) -> List[IdentifyResult]:
        identify = self.vision.get_onnx_identify_depth(True)

        result = []
        for i in identify:
            if fruit:
                if FruitType(i.class_id) not in fruit:
                    continue
            if i.distance > self.max_grab_distance:
                continue
            result.append(i)
        return result

    def grab_apple_from_tree(self):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓苹果!")
            return False

        self.close_tree()
        self.move.rotation_correction()
        ArmMovement.identify_tree_fruit(self.arm, self.direction)
        fruits = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
        prev_move_len = 0

        if not fruits:
            self.logger.warn("没有检测到水果!")
            ArmMovement.motion(self.arm)
            return False
        else:
            self.logger.info(f"检测到 {len(fruits)} 个水果")

        max_distance = max(self.find_fruits(), key=lambda fruit: fruit.distance).distance

        fruits.sort(key=lambda fruit: fruit.distance)
        for i in fruits:
            if i.distance == -1:
                self.logger.warn(f"没有深度信息，跳过{i.class_id}")
                continue

            if i.box.get_area() < 2000:
                self.logger.warn(f"面积 {i.box.get_area()} 太小，可能有遮挡，跳过 {i.class_id}")

            if max_distance - i.distance > 20:
                self.logger.warn(f"与可见的最大深度差值过大，不可抓，跳过 {i.class_id}")
                continue

            center = i.box.get_rectangle_center()
            fruit_type = FruitType(i.class_id)

            if self.direction == Direction.LEFT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, i.distance)
            elif self.direction == Direction.RIGHT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, i.distance)
            else:
                raise ValueError()
            move_distance = move_distance + 0.215

            self.logger.info(f"准备抓取: {i.class_id}, 距离: {i.distance}, 移动: {move_distance}")

            self.move.line(move_distance - prev_move_len, is_block=False)
            # # 补偿一些误差
            # prev_move_len = move_distance + 0.01

            ArmMovement.grab_apple_on_tree(self.arm, self.direction, (i.distance - 0.28) * 100, center.y > 160)
            for j in range(1, 4):
                basket = getattr(self, f"basket_{j}")
                if fruit_type in basket:
                    ArmMovement.put_fruit_to_basket(self.arm, j)
                    basket.remove(fruit_type)
                    self.arm.lift(0)
                    break

            if not self.has_apple():
                self.logger.info("全部抓取完成，停止抓取")
                break

        ArmMovement.motion(self.arm)
        return True

    def close_tree(self):
        """
        使用激光雷达找树，并靠近到想要的位置
        """

        if not self.direction:
            self.logger.warn("方向未设置。无法靠近苹果树!")
            return

        # 获取指定角度范围内距离最小点
        start_angle = 180 if self.direction == Direction.LEFT else 45
        radar_data = self.sensor.get_lidar_data(start_angle - 45, start_angle)
        min_tree = min(radar_data, key=lambda i: i[0])

        if not min_tree[0] or not min_tree[1]:
            self.logger.warn("无雷达数据!")
            return
        x, y = Math.polar_to_cartesian(min_tree)
        print(f"雷达数据: {min_tree}, 转换为坐标: ({x}, {y})")
        x -= 0.11
        y -= 0.52 if self.direction == Direction.LEFT else -0.52

        l, angle = Math.cartesian_to_polar((0, 0), (x, y))
        print(f"计算角度: {angle}. 计算距离: {l}")
        self.move.rotate(angle)
        self.move.line(l)
        self.move.rotate(-angle)
