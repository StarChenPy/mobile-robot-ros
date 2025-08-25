import time

from ..param import ArmMovement
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TaskCController:
    def __init__(self, node):
        self.logger = Logger()
        self.node = node

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)

    def run(self):
        start_time = time.time()

        fruit = self.identify_fruit()
        self.grab_fruit(FruitType(fruit.class_id))
        self.nav_to_put_point()
        self.put_fruit()

        end_time = time.time()
        use_time = end_time - start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")

    def identify_fruit(self):
        self.move.my_navigation("identify_fruit")

        ArmMovement.identify_ground_fruit(self.arm)
        while True:
            fruit = self.vision.find_fruit(FruitType.all())

            if fruit:
                fruit = self.vision.find_fruit(FruitType.all())
                if fruit:
                    return fruit

    def grab_fruit(self, fruit_target: FruitType):
        self.move.my_navigation("grab_fruit")
        while True:
            fruit = self.vision.find_fruit([fruit_target])
            if fruit:
                fruit = self.vision.find_fruit([fruit_target])
                if fruit:
                    self.logger.info(f"找到了一个 {fruit.class_id} 水果")
                    center = fruit.box.get_rectangle_center()

                    distance = 0.41
                    if fruit.distance != -1:
                        distance = fruit.distance

                    # 先移动到水果前面，使其在夹爪下方
                    move_dis = Math.pixel_to_distance_from_center(center.y, distance)
                    self.move.line(move_dis + 0.05)
                    ArmMovement.open_half_gripper(self.arm)

                    # 计算水果的左右偏移
                    x_dis = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
                    photo_telescopic_len = 0.25

                    # 伸缩控制
                    telescopic_len = Math.calculate_hypotenuse(photo_telescopic_len, x_dis)
                    self.arm.telescopic_servo((telescopic_len - photo_telescopic_len) * 100 + 6)

                    # 计算旋转
                    rotary_angle = -Math.calculate_right_triangle_angle(x_dis, telescopic_len)
                    if abs(rotary_angle) > 30:
                        rotary_angle *= 1.1
                    self.arm.rotary_servo(90 - rotary_angle)
                    self.arm.rotate(180 + rotary_angle)

                    # 抓取
                    self.arm.lift(29)
                    time.sleep(0.3)
                    if "grape" in fruit.class_id:
                        ArmMovement.close_gripper_grape(self.arm)
                        ArmMovement.motion_grape(self.arm)
                    else:
                        ArmMovement.close_gripper_apple(self.arm)
                        ArmMovement.motion_apple(self.arm)

    def nav_to_put_point(self):
        pass

    def put_fruit(self):
        pass

