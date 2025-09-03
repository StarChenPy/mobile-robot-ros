import time
from typing import List

import rclpy.node

from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
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
        self.max_grab_distance = 0.48  # 最大抓取距离，单位米

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
        identify = self.vision.get_onnx_identify_depth(True, 35)

        result = []
        for i in identify:
            if fruit:
                if FruitType(i.class_id) not in fruit:
                    continue
            if i.distance > self.max_grab_distance:
                self.logger.info(f"检测到一个 {i.class_id}, 但是距离 {i.distance} 超出 {self.max_grab_distance}")
                continue
            result.append(i)
        return result

    def grab_apple_from_tree(self):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓苹果!")
            return False

        self.close_tree()
        ArmMovement.identify_tree_fruit(self.arm, self.direction)
        time.sleep(1)
        fruits_1 = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
        fruits_2 = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
        while len(fruits_1) != len(fruits_2):
            fruits_1 = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
            fruits_2 = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
            self.logger.warn("两次拍摄水果数量不相同，重试.")
        fruits = fruits_2

        if not fruits:
            self.logger.warn("没有检测到水果!")
            self.arm.plan_list(ArmMovement.motion())
            return False
        else:
            self.logger.info(f"检测到 {len(fruits)} 个水果")

        fruits.sort(key=lambda fruit: fruit.distance)
        prev_move_len = 0
        for i in fruits:
            if i.distance == -1:
                self.logger.warn(f"{i.class_id} 没有深度信息，跳过")
                continue

            if FruitType(i.class_id) not in (self.basket_1 + self.basket_2 + self.basket_3):
                self.logger.info(f"已经抓够的的水果，跳过 {i.class_id}")
                continue

            if i.box.get_area() < 3000:
                self.logger.warn(f"面积 {i.box.get_area()} 太小，可能有遮挡或过于遥远，跳过 {i.class_id}")
                continue

            if i.distance > self.max_grab_distance:
                self.logger.warn(f"距离过远 {i.distance}，不可抓，跳过 {i.class_id}")
                continue

            center = i.box.get_rectangle_center()
            fruit_type = FruitType(i.class_id)

            if self.direction == Direction.LEFT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, i.distance)
            elif self.direction == Direction.RIGHT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, i.distance)
            else:
                raise ValueError()
            # move_distance = move_distance * 1.4 if move_distance < 0 else move_distance
            move_distance = move_distance * 1.15 + 0.215

            actual_move_len = move_distance - prev_move_len
            self.logger.info(f"准备抓取: {i.class_id} ({center.x}, {center.y}), 距离: {i.distance}, 移动: {actual_move_len}")

            self.move.line(actual_move_len, is_block=False)
            prev_move_len = move_distance

            ArmMovement.grab_apple_on_tree(self.arm, self.direction, (i.distance - 0.27) * 100, center.y > 180)
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

        self.arm.plan_list(ArmMovement.motion())
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
        self.logger.info(f"雷达数据: {min_tree}, 转换为坐标: ({x}, {y})")
        x -= 0.11
        y -= 0.56 if self.direction == Direction.LEFT else -0.56

        l, angle = Math.cartesian_to_polar((0, 0), (x, y))
        self.logger.info(f"计算角度: {angle}. 计算距离: {l}")

        if abs(l) < 0.02:
            self.logger.info(f"距离过短，跳过矫正.")
            return

        if abs(angle) > 90:
            angle = Math.normalize_angle(angle - 180)
            l = -l
        self.move.rotate(angle)
        self.move.line(l)
        self.move.rotate(-angle)
