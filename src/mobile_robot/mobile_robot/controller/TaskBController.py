import time

from ..param import NavMovement, ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math, Util
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TaskBController:
    def __init__(self, node):
        self.logger = Logger()

        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

    def run(self):
        self.robot.with_robot_connect()
        self.robot.set_start_led(False)
        i = input("从哪开始？ 1-10 \n")

        try:
            i = int(i)
            if i < 1 or i > 10:
                i = 1
        except ValueError:
            self.logger.warn("不是数字！")
            i = 1

        self.arm.back_origin()
        ArmMovement.motion(self.arm)

        for i in range(i, 11):
            # 动态获取任务，拒绝重复代码
            task_method = getattr(self, f"task_{i}")
            if callable(task_method):
                self.logger.info(f"开始执行任务B-任务{i}")
                self.robot.set_start_led(True)
                task_method()
                self.robot.set_start_led(False)
                self.logger.info(f"任务B-任务{i}完成")
            else:
                self.logger.error(f"任务B-任务{i}不存在")
            input("输入回车键继续...")

    def task_1(self):
        # 去黄站台2识别水果
        NavMovement.start_to_yellow_station_2(self.move)
        ArmMovement.identify_station_fruit_front(self.arm)
        yellow_station = self.vision.get_onnx_identify()[0]
        ArmMovement.motion(self.arm)

        # 去红站台1抓取水果
        NavMovement.yellow_station_2_to_red_station_1(self.move)
        self.move.rotation_correction()

        from_wall = self.sensor.get_distance_from_wall(Direction.FRONT)
        self.move.line(from_wall - 0.5)

        self.move.line(-0.10)
        ArmMovement.identify_station_fruit_front(self.arm)
        red_station = self.vision.get_onnx_identify()
        self.move.line(0.15)
        red_station.sort(key=lambda x: x.box.get_rectangle_center().x)

        if len(red_station) != 3:
            ArmMovement.grab_center_fruit_from_station(self.arm, 3)
            ArmMovement.grab_motion(self.arm, 3)
        elif red_station[0].class_id == yellow_station.class_id:
            ArmMovement.grab_left_fruit_from_station(self.arm, 3)
            ArmMovement.grab_motion(self.arm, 3)
        elif red_station[1].class_id == yellow_station.class_id:
            ArmMovement.grab_center_fruit_from_station(self.arm, 3)
            ArmMovement.grab_motion(self.arm, 3)
        elif red_station[2].class_id == yellow_station.class_id:
            ArmMovement.grab_right_fruit_from_station(self.arm, 3)
            ArmMovement.grab_motion(self.arm, 3)

    def find_fruit(self, fruit=None) -> IdentifyResult | None:
        identify = self.vision.get_onnx_identify()
        for i in identify:
            if fruit:
                if i.class_id not in fruit:
                    continue
            return i
        return None

    def task_2(self):
        # 去黄平台1抓取果篮
        NavMovement.start_to_yellow_station_2(self.move)
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)

        # 去葡萄园抓取3个地上的葡萄放到框子里
        NavMovement.yellow_station_2_to_vineyard(self.move)
        ArmMovement.identify_ground_fruit(self.arm)
        self.move.navigation([], 0.05, False)
        while self.move.get_status():
            fruit = self.find_fruit()
            if fruit:
                self.move.stop_navigation()
                fruit = self.find_fruit()
                if fruit:
                    center = fruit.box.get_rectangle_center()

                    # 先移动到水果前面，使其在夹爪下方
                    camera_to_claws = 0.05
                    move_dis = Math.pixel_to_distance_from_center(center.y, 0.41)
                    self.move.line(move_dis + camera_to_claws)

                    # 计算水果的左右偏移
                    x_dis = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, 0.41)
                    photo_telescopic_len = 0.25

                    # 伸缩控制
                    telescopic_len = Math.calculate_hypotenuse(photo_telescopic_len, x_dis)
                    self.arm.telescopic_servo((telescopic_len - photo_telescopic_len) * 100 + 6)

                    # 计算旋转
                    rotary_angle = -Math.calculate_right_triangle_angle(x_dis, telescopic_len)
                    if 180 + rotary_angle > 200:
                        self.arm.rotary_servo(-90 - rotary_angle)
                    else:
                        self.arm.rotary_servo(90 - rotary_angle)
                    self.arm.rotate(180 + rotary_angle, 40)

                    # 抓取
                    self.arm.lift(31, 40)
                    ArmMovement.close_gripper(self.arm, 4)
                    self.arm.lift(0, 40)

                    ArmMovement.put_fruit_to_basket(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 前往黄平台3放置果篮
        self.move.navigation([])
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.FRONT)
        ArmMovement.motion(self.arm)

    def task_3(self):
        # 去果树1抓一颗苹果
        self.move.navigation([])
        ArmMovement.identify_tree_fruit(self.arm)
        identify = self.vision.get_onnx_identify()
        fruit_location = Util.get_fruit_location(identify, [FruitType.RED_APPLE])
        if fruit_location:
            ArmMovement.top_180(self.arm)
            self.move.line(0.3)
            for i in fruit_location.keys():
                ArmMovement.grab_apple_on_tree(self.arm, i)
            ArmMovement.motion(self.arm)
            return

        # 如果正面没有，去侧面看一下
        ArmMovement.motion(self.arm)
        self.move.navigation([])
        ArmMovement.identify_tree_fruit(self.arm)
        identify = self.vision.get_onnx_identify()
        fruit_location = Util.get_fruit_location(identify, [FruitType.RED_APPLE])
        ArmMovement.top_180(self.arm)
        self.move.line(0.3)
        for i in fruit_location.keys():
            ArmMovement.grab_apple_on_tree(self.arm, i)
        ArmMovement.motion(self.arm)

    def task_4(self):
        # 前往葡萄园
        self.move.navigation([])

        # 寻找葡萄并抓取
        ArmMovement.identify_grape(self.arm, Direction.LEFT)
        self.move.navigation([NavigationPoint(1.0, 0, 0)], 0.05, False)
        while self.move.get_status():
            grape = self.find_fruit(["purple_grapes"])
            if grape:
                self.move.stop_navigation()
                grape = self.find_fruit(["purple_grapes"])
                if grape:
                    center = grape.box.get_rectangle_center()
                    x_dis = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, 0.23)

                    self.move.line(x_dis)

                    ArmMovement.grab_grape_on_wall(self.arm, Direction.LEFT, center.y > 280)
                    ArmMovement.top_180(self.arm)
        ArmMovement.motion(self.arm)

    def task_5(self):
        # 去抓第1个框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去抓第2个框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去抓第3个框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 3)
        ArmMovement.motion(self.arm)

        # 去放第1个框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.FRONT)
        ArmMovement.motion(self.arm)

        # 去放第2个框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_robot(self.arm, 2)
        ArmMovement.put_basket_to_station(self.arm, Direction.FRONT)
        ArmMovement.motion(self.arm)

        # 去放第3个框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_robot(self.arm, 3)
        ArmMovement.put_basket_to_station(self.arm, Direction.FRONT)
        ArmMovement.motion(self.arm)

        ArmMovement.motion(self.arm)

    def task_6(self):
        # 去黄站台1抓一个篮子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去苹果区抓一个苹果，放到篮子里
        self.move.navigation([])
        ArmMovement.grab_apple_on_tree(self.arm, FruitLocationOnTree.TOP_CENTER)
        ArmMovement.put_fruit_to_basket(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去葡萄区抓一个葡萄，放到篮子里
        self.move.navigation([])
        ArmMovement.grab_grape_on_wall(self.arm, Direction.FRONT)
        ArmMovement.put_fruit_to_basket(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 将果篮放在红站台1
        self.move.navigation([])
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.FRONT)
        ArmMovement.motion(self.arm)

        # 回到起始区
        self.move.navigation([])

    def task_7(self):
        # 去黄站台2抓框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去红站台1抓框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去红站台2抓框子
        self.move.navigation([])
        ArmMovement.grab_basket_from_station(self.arm, Direction.FRONT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去果树2抓6颗水果，分别为红绿、绿黄、黄紫
        self.move.navigation([])
        ArmMovement.identify_tree_fruit(self.arm)
        identify = self.vision.get_onnx_identify()
        basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]
        fruit_location = Util.get_fruit_location(identify, basket_1 + basket_2 + basket_3)

        if fruit_location:
            ArmMovement.top_180(self.arm)
            self.move.line(0.3)
            for k, v in fruit_location.items():
                if v in basket_1:
                    ArmMovement.grab_apple_on_tree(self.arm, k)
                    ArmMovement.put_fruit_to_basket(self.arm, 1)
                    basket_1.remove(v)
                elif v in basket_2:
                    ArmMovement.grab_apple_on_tree(self.arm, k)
                    ArmMovement.put_fruit_to_basket(self.arm, 2)
                    basket_2.remove(v)
                elif v in basket_3:
                    ArmMovement.grab_apple_on_tree(self.arm, k)
                    ArmMovement.put_fruit_to_basket(self.arm, 2)
                    basket_3.remove(v)
        ArmMovement.motion(self.arm)

        # 如果还有水果没抓，去侧面看看
        if basket_1 or basket_2 or basket_3:
            self.move.navigation([])
            ArmMovement.identify_tree_fruit(self.arm)
            identify = self.vision.get_onnx_identify()
            fruit_location = Util.get_fruit_location(identify, basket_1 + basket_2 + basket_3)
            if fruit_location:
                ArmMovement.top_180(self.arm)
                self.move.line(0.3)
                for k, v in fruit_location.items():
                    if v in basket_1:
                        ArmMovement.grab_apple_on_tree(self.arm, k)
                        ArmMovement.put_fruit_to_basket(self.arm, 1)
                        basket_1.remove(v)
                    elif v in basket_2:
                        ArmMovement.grab_apple_on_tree(self.arm, k)
                        ArmMovement.put_fruit_to_basket(self.arm, 2)
                        basket_2.remove(v)
                    elif v in basket_3:
                        ArmMovement.grab_apple_on_tree(self.arm, k)
                        ArmMovement.put_fruit_to_basket(self.arm, 2)
                        basket_3.remove(v)
            ArmMovement.motion(self.arm)

        # 回起始区
        self.move.navigation([])

