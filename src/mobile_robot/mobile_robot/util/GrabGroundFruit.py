import time

import rclpy.node

from . import Math
from ..param import ArmMovement, RobotConstant
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from .Logger import Logger


class GrabGroundFruit:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)
        self.sensor = SensorService(node)

        self.goal_num = 0

        self.fruit_type = []

    def patrol_line(self, goal: str) -> bool:
        """
        向指定的点位导航，并寻找地上的水果
        @return 是否在导航结束前成功抓取
        """
        if not self.scan_corridor():
            self.logger.info("没有在这条走廊找到水果，跳过.")
            return False

        if self.goal_num <= 0:
            self.logger.info("没有可抓的水果，跳过.")
            return False

        ArmMovement.identify_ground_fruit(self.arm)
        self.move.my_navigation(goal, 0.15, start_name=goal, block=False)
        while self.move.get_my_status():
            fruit = self.vision.find_fruit(FruitType.all())
            if fruit:
                self.move.stop_my_navigation()
                time.sleep(0.5)

                if self.grab_ground_fruit():
                    ArmMovement.put_fruit_to_basket(self.arm, 2)

                    # 判断一下是否达到目标要求
                    self.goal_num -= 1
                    if self.goal_num <= 0:
                        self.move.my_navigation(goal, 0.6, start_name=goal)
                        return True

                    ArmMovement.identify_ground_fruit(self.arm, 60)

                self.move.my_navigation(goal, 0.15, start_name=goal, block=False)

        return False

    def scan_corridor(self) -> bool:
        """
        在扫地之前，先看看走廊中有没有水果
        """

        ArmMovement.identify_ground_fruit(self.arm, 50)
        fruit = self.vision.find_fruit(FruitType.all(), kernel_size=75)
        if fruit:
            return True
        else:
            self.arm.servo_nod(-5)
            time.sleep(1)
            fruit = self.vision.find_fruit(FruitType.all(), kernel_size=75)
            if fruit:
                if fruit.distance != -1:
                    # 这里不太好，先这么写着
                    self.move.line(fruit.distance * 0.9, speed=0.6)
                return True
        return False

    def grab_ground_fruit(self, fruit_types=None) -> bool:
        """
        识别地上的水果并抓取
        @return 是否成功抓取
        """
        if fruit_types is None:
            fruit_types = FruitType.all()

        fruit = self.vision.find_fruit(fruit_types)
        if fruit:
            self.logger.info(f"找到了一个 {fruit.class_id} 水果")
            center = fruit.box.get_rectangle_center()

            distance = 0.36

            # 先移动到水果前面，使其在相机下方
            move_dis = Math.pixel_to_distance_from_center(center.y, distance)
            self.move.line(move_dis)

            # 计算水果的左右偏移
            x_dis = Math.pixel_to_horizontal_distance_x_centered(RobotConstant.CAMERA_WIDTH / 2 - center.x, distance)
            photo_telescopic_len = 0.32

            # 伸缩控制
            telescopic_len = Math.calculate_hypotenuse(photo_telescopic_len, x_dis)

            # 计算旋转
            rotary_angle = -Math.calculate_right_triangle_angle(x_dis, photo_telescopic_len)

            # 修复一下因为夹爪特性导致的多余数值
            rotary_angle *= 0.8

            is_small = "grape" in fruit.class_id or "kiwifruit" in fruit.class_id or "lotus_fruit" in fruit.class_id

            ArmMovement.grab_fruit_from_ground(self.arm, rotary_angle, ((telescopic_len - photo_telescopic_len) * 100) + 10, is_small)

            return True
        return False