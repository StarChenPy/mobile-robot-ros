import time

from ..param import ArmMovement
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Utils
from ..util.GrabGroundFruit import GrabGroundFruit
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
        self.arm.plan_list(ArmMovement.motion())
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

    def identify_fruit(self):
        """
        在起始区直接向后看识别水果
        """

        self.arm.servo_nod(0)
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
        ground_fruit = GrabGroundFruit(self.node)
        ArmMovement.identify_ground_fruit(self.arm, 60)
        while not ground_fruit.grab_ground_fruit([self.fruit]):
            self.logger.warn("没有找到水果")
            pass
        ArmMovement.motion_apple(self.arm)
        self.move.rotate(90)
        self.logger.info("准备开始走迷宫")

    def nav_to_put(self):
        for i in range(3):
            if Utils.jx_nav2().success:
                self.arm.plan_list(ArmMovement.put_fruit_to_ground())
                self.arm.plan_list(ArmMovement.motion())
                return
            else:
                self.logger.warn(f"导航到放置点失败，重试第 {i} 次.")
                time.sleep(1)
        self.logger.error("导航放置点失败.")
