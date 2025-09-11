import time
from typing import List

import rclpy.node

from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement, RobotConstant
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
        """
        判断框子中是否还有苹果
        """

        apple_types = set(FruitType.apples())
        for i in range(1, 4):
            basket = getattr(self, f"basket_{i}")
            if basket and any(fruit in apple_types for fruit in basket):
                return True
        return False

    def apples_in_basket(self, apple: FruitType | str) -> bool:
        """
        判断水果在不在需求列表中
        :param apple: 苹果，可以是名称或是FruitType
        """

        if isinstance(apple, str):
            apple = FruitType(apple)

        basket = self.basket_1 + self.basket_2 + self.basket_3
        return apple in basket

    def get_grape_in_which_basket(self, apple: FruitType | str) -> int:
        """
        获取给定的苹果所在的框子编号
        """

        if isinstance(apple, str):
            apple = FruitType(apple)

        for i in range(1, 4):
            bastet = getattr(self, f"basket_{i}")
            if apple in bastet:
                return i
        return 0

    def find_apples_you_need(self) -> list[IdentifyResult]:
        """
        获取所需的葡萄识别结果
        """

        def get_result() -> list[IdentifyResult]:
            identify = self.vision.get_onnx_identify_depth(True, 95)
            return [i for i in identify if self.apples_in_basket(i.class_id)]

        # 第一次结果
        result = get_result()

        while True:
            new_result = get_result()
            if len(result) == len(new_result):
                return result  # 数量一致，返回
            result = new_result  # 更新结果，继续循环

    def grab_apple_from_tree(self):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓苹果!")
            return False

        self.close_tree()
        ArmMovement.identify_tree_fruit(self.arm, self.direction)
        time.sleep(1)
        fruits = self.find_apples_you_need()

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
                width_center = RobotConstant.CAMERA_WIDTH / 2
                if width_center - 100 < i.box.get_rectangle_center().x < width_center + 100:
                    self.logger.warn(f"{i.class_id} 没有深度信息，跳过")
                else:
                    self.logger.warn(f"{i.class_id} 没有深度信息，但在画面中间，使用默认深度")
                    i.distance = 0.25
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
            is_low = center.y > 180
            extra_swing_angle = 0

            if self.direction == Direction.LEFT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, i.distance)
                if is_low:
                    extra_swing_angle = 20 if move_distance > 0 else -20
            elif self.direction == Direction.RIGHT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, i.distance)
                if is_low:
                    extra_swing_angle = -20 if move_distance > 0 and is_low else 20
            else:
                raise ValueError()
            move_distance = move_distance + 0.215

            actual_move_len = move_distance - prev_move_len
            if abs(actual_move_len) > 0.1:
                actual_move_len *= 1.1
            self.logger.info(f"准备抓取: {i.class_id} ({center.x}, {center.y}), 距离: {i.distance}, 移动: {actual_move_len}")

            self.move.line(actual_move_len, is_block=False)
            prev_move_len = move_distance

            ArmMovement.grab_apple_on_tree(self.arm, self.direction, (i.distance - 0.26) * 100, is_low, extra_swing_angle)
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

    def get_lidar_min_point(self) -> tuple[float, float]:
        # 获取指定角度范围内距离最小点
        rclpy.spin_once(self.node)
        time.sleep(0.3)
        rclpy.spin_once(self.node)
        start_angle = 178 if self.direction == Direction.LEFT else 32
        radar_data = self.sensor.get_lidar_data(start_angle - 30, start_angle)
        min_tree_1 = min(radar_data, key=lambda i: i[0])

        time.sleep(0.3)
        rclpy.spin_once(self.node)

        radar_data = self.sensor.get_lidar_data(start_angle - 30, start_angle)
        min_tree_2 = min(radar_data, key=lambda i: i[0])

        if abs(min_tree_1.distance - min_tree_2.distance) < 0.03:
            return min_tree_2
        else:
            return 0, 0


    def close_tree(self):
        """
        使用激光雷达找树，并靠近到想要的位置
        """

        if not self.direction:
            self.logger.warn("方向未设置。无法靠近苹果树!")
            return

        # 获取指定角度范围内距离最小点
        rclpy.spin_once(self.node)
        time.sleep(0.3)
        rclpy.spin_once(self.node)
        start_angle = 178 if self.direction == Direction.LEFT else 32
        radar_data = self.sensor.get_lidar_data(start_angle - 30, start_angle)
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
        elif abs(l) > 0.2:
            self.logger.info("距离过远，不可信，跳过矫正.")
            return

        if abs(angle) > 90:
            angle = Math.normalize_angle(angle - 180)
            l = -l
        self.move.rotate(angle)
        self.move.line(l)
        self.move.rotate(-angle)
