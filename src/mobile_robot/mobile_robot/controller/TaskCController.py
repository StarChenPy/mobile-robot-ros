import time

import paramiko

from ..param import ArmMovement
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
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
        self.sensor = SensorService(node)
        self.arm = ArmService(node)
        self.robot = RobotService(node)
        self.vision = VisionService(node)

        self.fruit = None

    def run(self):
        self.robot.with_robot_connect()
        self.robot.set_start_led(True)
        self.arm.back_origin()
        ArmMovement.motion(self.arm)
        self.robot.with_start_button()

        start_time = time.time()

        self.sensor.init_odom_all(NavigationPoint(0.9, 0.3, 0))
        # 原地识别水果
        self.identify_fruit()
        # 去抓取水果
        self.nav_to_grab()
        # 去放置水果
        self.nav_to_put()

        end_time = time.time()
        use_time = end_time - start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")

    def grab_ground_fruit(self) -> bool:
        fruit = self.vision.find_fruit([self.fruit])
        if fruit:
            self.logger.info(f"找到了一个 {fruit.class_id} 水果")
            center = fruit.box.get_rectangle_center()

            distance = 0.41
            if fruit.distance != -1:
                distance = fruit.distance

            # 先移动到水果前面，使其在夹爪下方
            move_dis = Math.pixel_to_distance_from_center(center.y, distance)
            self.move.line(move_dis + 0.03)
            ArmMovement.open_half_gripper(self.arm)

            # 计算水果的左右偏移
            x_dis = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
            photo_telescopic_len = 0.24

            # 伸缩控制
            telescopic_len = Math.calculate_hypotenuse(photo_telescopic_len, x_dis)
            self.arm.telescopic_servo((telescopic_len - photo_telescopic_len) * 100 + 6)

            # 计算旋转
            rotary_angle = -Math.calculate_right_triangle_angle(x_dis, telescopic_len)
            if abs(rotary_angle) > 30:
                rotary_angle *= 1.2
            self.arm.rotary_servo(90 - rotary_angle)
            self.arm.rotate(180 + rotary_angle)

            # 抓取
            self.arm.nod_servo(90)
            self.arm.lift(29)
            time.sleep(0.3)
            if "grape" in fruit.class_id:
                ArmMovement.close_gripper_grape(self.arm)
            else:
                ArmMovement.close_gripper_apple(self.arm)
            self.arm.lift(0)

            ArmMovement.put_fruit_to_basket(self.arm, 2)

            return True
        return False

    def put_fruit(self):
        self.arm.rotate(180, is_block=False)
        self.arm.lift(30, is_block=False)
        self.arm.nod_servo(90)
        self.arm.telescopic_servo(15)
        self.arm.wait_finish()
        ArmMovement.open_gripper(self.arm)

    def identify_fruit(self):
        """
        在起始区直接向后看识别水果
        """

        self.arm.nod_servo(0)
        time.sleep(2)

        while True:
            fruit = self.vision.find_fruit(FruitType.all())

            if fruit:
                self.logger.info(f"识别到一个 {fruit.class_id} 水果, 再次确认")
                fruit = self.vision.find_fruit(FruitType.all())
                if fruit:
                    self.logger.info(f"确认识别到一个 {fruit.class_id} 水果")
                    self.fruit = FruitType(fruit.class_id)
                    return

    def nav_to_grab(self):
        self.logger.info("导航到抓取点位")
        self.move.navigation([NavigationPoint(3.3, 0.3, 0)])
        self.logger.info("开始抓取")
        ArmMovement.identify_ground_fruit(self.arm)
        while not self.grab_ground_fruit():
            self.logger.warn("没有找到水果")
            pass
        self.move.navigation([NavigationPoint(3.3, 0.3, 90)])
        self.logger.info("准备开始走迷宫")

    def nav_to_put(self):
        self.move.jx_nav2()
        self.put_fruit()
