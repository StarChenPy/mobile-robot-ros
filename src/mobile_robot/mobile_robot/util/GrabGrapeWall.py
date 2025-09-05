import time

from rclpy.node import Node

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
class GrabGrapeWall:
    def __init__(self, node: Node):
        self.logger = Logger()
        self.node = node

        self.continuous = False
        self.direction = None
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

    def find_fruits(self) -> list[IdentifyResult]:
        identify = self.vision.get_onnx_identify_depth(True, 95)

        result = []
        for i in identify:
            self_basket = self.basket_1 + self.basket_2 + self.basket_3
            if self_basket:
                if FruitType(i.class_id) not in self_basket:
                    continue
            result.append(i)
        return result

    def grab_grape_from_wall(self, continuous=False):
        """
        识别一面墙上的葡萄，然后抓取它们
        :param continuous: 是否连续作业，若为True，则抓取结束后继续调整为识别姿态
        :return: None
        """

        if self.direction is None:
            self.logger.error("方向未设置。无法抓苹果!")

        # 直角矫正
        self.move.rotation_correction(self.direction.invert())

        # 判断是否要到识别姿态
        if self.continuous and continuous:
            self.arm.wait_plan_finish()
        else:
            self.arm.plan_list(ArmMovement.identify_grape(self.direction))
            self.continuous = continuous

        # 拍照并校验
        fruits_1 = self.find_fruits()
        fruits_2 = self.find_fruits()
        while len(fruits_1) != len(fruits_2):
            fruits_1 = self.find_fruits()
            fruits_2 = self.find_fruits()
            self.logger.warn("两次拍摄水果数量不相同，重试.")
        fruits = fruits_2

        if not fruits:
            self.logger.warn("没有检测到水果!")
        else:
            self.logger.info(f"检测到 {len(fruits)} 个水果")

            fruits.sort(key=lambda fruit: fruit.box.get_rectangle_center().x, reverse=self.direction == Direction.RIGHT)
            prev_move_len = 0
            for i in fruits:
                if FruitType(i.class_id) not in (self.basket_1 + self.basket_2 + self.basket_3):
                    self.logger.info(f"已经抓够的的水果，跳过 {i.class_id}")
                    continue

                if i.box.get_area() < 4000:
                    self.logger.warn(f"面积 {i.box.get_area()} 太小，可能有遮挡或过于遥远，跳过 {i.class_id}")
                    continue

                center = i.box.get_rectangle_center()
                fruit_type = FruitType(i.class_id)
                distance = i.distance if i.distance != -1 else 0.30

                if self.direction == Direction.LEFT:
                    move_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)
                elif self.direction == Direction.RIGHT:
                    move_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
                else:
                    raise ValueError()
                move_distance += 0.09 if self.direction == Direction.LEFT else 0.05

                actual_move_len = move_distance - prev_move_len
                self.logger.info(f"准备抓取: {i.class_id} ({center.x}, {center.y}), 距离: {distance}, 移动: {actual_move_len}")

                angle = Math.calculate_right_triangle_angle(distance - 0.02, 0.34)

                y_distance = Math.pixel_to_distance_from_bottom(center.y, distance)
                lift_height = (40 if distance != 0.3 else 34) - (y_distance * 100)
                ir_distance = self.sensor.get_ir_left() if self.direction == Direction.LEFT else self.sensor.get_ir_right()
                if ir_distance:
                    if ir_distance > 0.37:
                        self.logger.info("离葡萄墙距离远，额外抬高夹爪，降低升降.")
                        nod = 45
                        lift_height += 5
                    else:
                        self.logger.info("离葡萄墙距离近，使用默认方案.")
                        nod = 50
                else:
                    self.logger.info("无红外数据，使用默认方案.")
                    nod = 50
                self.arm.plan_list(ArmMovement.grab_grape_on_wall(self.direction, lift_height, angle, nod), block=False)

                self.move.line(actual_move_len, is_block=False)
                self.arm.wait_plan_finish()
                prev_move_len = move_distance

                for j in range(1, 4):
                    basket = getattr(self, f"basket_{j}")
                    if fruit_type in basket:
                        ArmMovement.put_fruit_to_basket(self.arm, j, True)
                        basket.remove(fruit_type)
                        break

                if not self.has_grape():
                    self.logger.info("全部抓取完成，停止抓取")
                    break

        if continuous:
            self.arm.plan_list(ArmMovement.identify_grape(self.direction), block=False)
        else:
            self.arm.plan_list(ArmMovement.motion(), block=False)

    def grab_grape(self, grape: IdentifyResult):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓葡萄!")
            return

        if not grape:
            self.logger.error("识别结果为空!")
            return

        center = grape.box.get_rectangle_center()

        distance = grape.distance if grape.distance != -1 else 0.30
        self.logger.info(f"抓取葡萄，距离: {distance}m, 像素坐标: ({center.x}, {center.y})")

        x_distance = 0.07 if self.direction == Direction.LEFT else 0.02
        if self.direction == Direction.LEFT:
            x_distance += Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)
        elif self.direction == Direction.RIGHT:
            x_distance += Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
        y_distance = Math.pixel_to_distance_from_bottom(center.y, distance)
        self.move.line(x_distance)

        # distance要加上夹爪的5cm, x_distance要加上从夹爪到旋转中心的34cm
        angle = Math.calculate_right_triangle_angle(distance, 0.34)

        lift_height = 32 - (y_distance * 100)

        self.arm.plan_list(ArmMovement.grab_grape_on_wall(self.direction, lift_height, angle, 60))

    def patrol_line(self, start_name: str, gaol_name: str):
        if self.direction is None:
            self.logger.error("方向未设置。无法抓葡萄!")
            return

        self.move.my_navigation(start_name)

        self.arm.plan_list(ArmMovement.identify_grape(self.direction))

        self.move.my_navigation(gaol_name, 0.1, gaol_name, block=False)
        while self.move.get_my_status():
            # 因为葡萄的深度比较难以检测，所以扩大深度获取的范围
            grape = self.vision.find_fruit(self.basket_1 + self.basket_2 + self.basket_3, True, 75)
            if not grape:
                continue

            # 检测到了，就停下来再看一遍
            self.move.stop_my_navigation()
            time.sleep(0.5)
            self.move.rotation_correction(Direction.FRONT, True, 10)
            time.sleep(0.5)
            grape = self.vision.find_fruit(self.basket_1 + self.basket_2 + self.basket_3, True, 75)
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
                self.arm.plan_list(ArmMovement.identify_grape(self.direction))

            # 如果框子里没有要抓的水果了，直接返回
            if not self.has_grape():
                self.logger.info("框子里没有要抓的葡萄了，停止抓取。")
                break

            # 继续导航
            self.move.my_navigation(gaol_name, 0.1, start_name, block=False)

        self.arm.plan_list(ArmMovement.motion())
