import time

import rclpy.node

from . import Math
from ..param import ArmMovement
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

        self.fruit_type = []

    def patrol_line_ground_fruit(self, goal: str) -> bool:
        """
        向指定的点位导航，并寻找地上的水果
        @return 是否在导航结束前成功抓取
        """

        ArmMovement.identify_ground_fruit(self.arm)
        self.move.my_navigation(goal, 0.2, start_name=goal, block=False)
        while self.move.get_my_status():
            fruit = self.vision.find_fruit(FruitType.all())
            if fruit:
                self.move.stop_my_navigation()
                time.sleep(0.5)
                # 如果抓取失败，继续巡线
                if self.grab_ground_fruit():
                    ArmMovement.put_fruit_to_basket(self.arm, 2)
                    self.move.rotation_correction(Direction.FRONT, True, 10)
                    return True
                else:
                    self.move.my_navigation(goal, 0.2, start_name=goal, block=False)
        return False

    def grab_ground_fruit(self) -> bool:
        """
        识别地上的水果并抓取
        @return 是否成功抓取
        """

        fruit = self.vision.find_fruit(FruitType.all())
        if fruit:
            self.logger.info(f"找到了一个 {fruit.class_id} 水果")
            center = fruit.box.get_rectangle_center()

            distance = 0.41
            if fruit.distance != -1:
                distance = fruit.distance

            # 先移动到水果前面，使其在夹爪下方
            move_dis = Math.pixel_to_distance_from_center(center.y, distance)
            self.move.line(move_dis)

            # 计算水果的左右偏移
            x_dis = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
            photo_telescopic_len = 0.24

            # 伸缩控制
            telescopic_len = Math.calculate_hypotenuse(photo_telescopic_len, x_dis)
            self.arm.servo_telescopic((telescopic_len - photo_telescopic_len) * 100 + 6)

            # 计算旋转
            rotary_angle = -Math.calculate_right_triangle_angle(x_dis, telescopic_len)
            if abs(rotary_angle) > 30:
                rotary_angle *= 1.2

            ArmMovement.grab_fruit_from_ground(self.arm, rotary_angle, (telescopic_len - photo_telescopic_len) * 100 + 6, "grape" in fruit.class_id)

            return True
        return False