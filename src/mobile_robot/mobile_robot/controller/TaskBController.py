import time

from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.GrabAppleTree import GrabAppleTree
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.Logger import Logger
from ..util.Singleton import singleton
from ..util.Station import Station


@singleton
class TaskBController:
    def __init__(self, node):
        self.logger = Logger()

        self.node = node
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

        # 动态获取任务，拒绝重复代码
        task_method = getattr(self, f"task_{i}")
        if callable(task_method):
            self.logger.info(f"开始执行任务B-任务{i}")
            start_time = time.time()
            self.robot.set_start_led(True)
            self.sensor.correction("c_start")
            time.sleep(1)
            task_method()
            self.robot.set_start_led(False)
            end_time = time.time()
            use_time = end_time - start_time
            self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")
            self.logger.info(f"任务B-任务{i}完成")
        else:
            self.logger.error(f"任务B-任务{i}不存在")

    def task_1(self):
        # 去黄站台2识别水果
        self.logger.info("导航前往黄站台2")
        self.move.my_navigation("s_y_2_r")
        self.vision.set_other_fruit_weight()
        self.logger.info("开始识别水果")
        ArmMovement.identify_station_fruit(self.arm, Direction.RIGHT)
        yellow_station = None
        while yellow_station is None:
            yellow_station = self.vision.find_fruit(FruitType.all(), True)
            time.sleep(1)
        self.logger.info("识别成功，准备移动至红站台1")
        ArmMovement.motion(self.arm)

        # 去红站台1抓取水果
        self.move.my_navigation("c_8")
        self.move.my_navigation("s_r_1_r")

        self.logger.info("开始识别水果")
        ArmMovement.identify_station_fruit(self.arm, Direction.RIGHT)
        red_station = None
        while red_station is None:
            red_station = self.vision.find_fruit([FruitType(yellow_station.class_id)], True)
            time.sleep(1)
        move_dis = Math.pixel_to_horizontal_distance_x_centered(320 - red_station.box.get_rectangle_center().x, 0.2)
        self.logger.info(f"寻找到横向 {move_dis} 米的距离处有一个 {red_station.class_id} 水果")
        self.move.line(move_dis + 0.07)

        self.logger.info("准备抓取水果")
        ArmMovement.grab_fruit_from_station(self.arm, Direction.RIGHT, False)

        ArmMovement.motion_apple(self.arm)

    def patrol_line_ground_fruit(self, point: str) -> bool:
        ArmMovement.identify_ground_fruit(self.arm)
        self.move.my_navigation(point, 0.1, block=False)
        while self.move.get_my_status():
            fruit = self.vision.find_fruit(FruitType.all())
            if fruit:
                self.move.stop_my_navigation()
                time.sleep(1)
                # 如果抓取失败，继续巡线
                if self.grab_ground_fruit():
                    self.move.rotation_correction(Direction.FRONT, True, 10)
                    return True
                else:
                    self.move.my_navigation(point, 0.1, block=False)
        return False

    def grab_ground_fruit(self) -> bool:
        fruit = self.vision.find_fruit(FruitType.all())
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

    def task_2(self):
        self.vision.set_other_fruit_weight()

        # 去黄平台1抓取果篮
        self.logger.info("导航前往黄站台1")
        Station.YELLOW_1.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        def to_put_basket():
            """前往黄平台3放置果篮"""
            self.logger.info("抓取完毕，前往放置果篮")
            Station.YELLOW_3.nav_and_put(self.node, 2, True)
            ArmMovement.motion(self.arm)

        def grab_fruits(_path, target_waypoint, _count):
            """前往指定点抓水果，返回更新后的 count"""
            if _path:
                for r in _path:
                    self.move.my_navigation(r)
            while _count < 3:
                # 当走到头时会返回False，跳出循环进入下一个路径
                if self.patrol_line_ground_fruit(target_waypoint):
                    _count += 1
                else:
                    break
            self.arm.rotary_servo(0)
            ArmMovement.motion(self.arm)
            return _count

        # 依次尝试的路线
        plans = [
            (["c_6", "v_3"], "c_5"),
            (["v_6"], "v_5"),
            (["c_4", "v_7"], "v_1"),
            (["c_2"], "c_3"),
        ]

        count = 0
        for path, point in plans:
            self.logger.info(f"尝试扫描到 {point} 抓水果")
            count = grab_fruits(path, point, count)
            if count >= 3:
                break

        to_put_basket()

    def task_3(self):
        # 去果树1抓一颗苹果
        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE]

        button = self.robot.with_start_button()
        if button:
            self.logger.info("检测到按键，开始前往树1前")
            self.move.my_navigation("t_1_f")
            grab_apple_tree.direction = Direction.LEFT
            grab_apple_tree.grab_apple_from_tree()
            if not grab_apple_tree.has_apple():
                return

            self.logger.info("未检测到苹果，前往树1后")
            self.move.my_navigation("t_1_b")
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()
        else:
            self.logger.info("检测到按键，开始前往树1后")
            self.move.my_navigation("t_1_b")
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()
            if not grab_apple_tree.has_apple():
                return

            self.logger.info("未检测到苹果，前往树1前")
            self.move.my_navigation("t_1_f")
            grab_apple_tree.direction = Direction.LEFT
            grab_apple_tree.grab_apple_from_tree()

    def task_4(self):
        # 寻找葡萄并抓取
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE]
        grab_grape_wall.find_grape_and_grab("c_2", "c_3")

    def task_5(self):
        # 去抓第1个框子
        self.logger.info("导航前往黄站台2")
        Station.YELLOW_2.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去抓第2个框子
        self.logger.info("导航前往黄站台3")
        Station.YELLOW_3.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去抓第3个框子
        self.logger.info("导航前往黄站台1")
        Station.YELLOW_1.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 3)
        ArmMovement.motion(self.arm)

        # 去放第1个框子
        self.logger.info("导航前往红站台3")
        Station.RED_3.nav_and_put(self.node, 3)
        ArmMovement.motion(self.arm)

        # 去放第2个框子
        self.logger.info("导航前往红站台1")
        Station.RED_1.nav_and_put(self.node, 2)
        ArmMovement.motion(self.arm)

        # 去放第3个框子
        self.logger.info("导航前往红站台2")
        Station.RED_2.nav_and_put(self.node, 1)
        ArmMovement.motion(self.arm)

    def task_6(self):
        # 去黄站台1抓一个篮子
        Station.YELLOW_1.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去葡萄区抓一个葡萄，放到篮子里
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.RIGHT
        self.move.my_navigation("c_5")
        self.move.my_navigation("c_5")
        ArmMovement.identify_grape(self.arm, Direction.RIGHT)
        while True:
            fruit = self.vision.find_fruit(FruitType.grapes(), True, 71)
            if fruit:
                grab_grape_wall.grab_grape(fruit)
                ArmMovement.put_fruit_to_basket(self.arm, 2, True)
                ArmMovement.motion(self.arm)
                self.move.rotation_correction(Direction.FRONT, True)
                break
        time.sleep(1)

        # 去苹果区抓一个苹果，放到篮子里
        self.move.my_navigation("c_7")
        self.move.my_navigation("t_1_g")
        ArmMovement.grab_apple_on_tree(self.arm, Direction.RIGHT, 0, False)
        ArmMovement.put_fruit_to_basket(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 将果篮放在红站台1
        Station.RED_1.nav_and_put(self.node, 2)
        ArmMovement.motion(self.arm)

        # 回到起始区
        self.move.my_navigation("c_start")
        ArmMovement.end(self.arm)

    def task_7(self):
        # 去红站台2抓框子
        Station.RED_2.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去黄站台2抓框子
        Station.YELLOW_2.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去红站台1抓框子
        Station.RED_1.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 3)
        ArmMovement.motion(self.arm)

        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]
        # # 去果树2抓6颗水果，分别为红绿、绿黄、黄紫
        self.move.my_navigation("t_2_r")
        grab_apple_tree.direction = Direction.LEFT
        grab_apple_tree.grab_apple_from_tree()
        if grab_apple_tree.has_apple():
            self.move.my_navigation("c_7")
            self.move.my_navigation("c_8")
            self.move.my_navigation("t_2_l")
            grab_apple_tree.direction = Direction.LEFT
            grab_apple_tree.grab_apple_from_tree()

        # 回起始区
        self.move.my_navigation("c_8")
        self.move.my_navigation("c_start")
        ArmMovement.end(self.arm)

    def task_8(self):
        # 去黄站台2抓框子
        Station.YELLOW_2.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去黄站台3抓框子
        Station.YELLOW_3.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去黄站台1抓框子
        Station.YELLOW_1.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 3)
        ArmMovement.motion(self.arm)

        # 前往葡萄区抓取葡萄
        self.move.my_navigation("c_6")
        self.move.rotation_correction(Direction.RIGHT, True)
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.basket_1 = [FruitType.GREEN_GRAPE, FruitType.YELLOW_GRAPE]
        grab_grape_wall.basket_2 = [FruitType.YELLOW_GRAPE, FruitType.PURPLE_GRAPE]
        grab_grape_wall.basket_3 = [FruitType.PURPLE_GRAPE, FruitType.GREEN_GRAPE]
        grab_grape_wall.find_grape_and_grab("v_4", "c_6")

        # 返回起始区
        self.move.my_navigation("c_start")
        ArmMovement.end(self.arm)

    def task_9(self):
        # 去会动的果树1抓一颗苹果
        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE]

        button = self.robot.with_start_button()
        if button:
            self.logger.info("检测到按键，开始前往树1前")
            self.move.my_navigation("t_1_f")
            grab_apple_tree.direction = Direction.LEFT
            grab_apple_tree.grab_apple_from_tree()
            if not grab_apple_tree.has_apple():
                return

            self.logger.info("未检测到苹果，前往树1后")
            self.move.my_navigation("t_1_b")
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()
        else:
            self.logger.info("检测到按键，开始前往树1后")
            self.move.my_navigation("t_1_b")
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()
            if not grab_apple_tree.has_apple():
                return

            self.logger.info("未检测到苹果，前往树1前")
            self.move.my_navigation("t_1_f")
            grab_apple_tree.direction = Direction.LEFT
            grab_apple_tree.grab_apple_from_tree()

    def task_10(self):
        # 去红站台2抓框子
        Station.RED_2.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去黄站台2抓框子
        Station.YELLOW_2.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去红站台1抓框子
        Station.RED_1.nav_and_grab(self.node)
        ArmMovement.put_basket_to_robot(self.arm, 3)
        ArmMovement.motion(self.arm)

        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]
        # # 去果树3抓6颗水果，分别为红绿、绿黄、黄紫
        self.move.my_navigation("t_3_r")
        grab_apple_tree.direction = Direction.RIGHT
        grab_apple_tree.grab_apple_from_tree()
        if grab_apple_tree.has_apple():
            self.move.my_navigation("c_7")
            self.move.my_navigation("c_8")
            self.move.my_navigation("t_3_l")
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()

        # 回起始区
        self.move.my_navigation("c_8")
        self.move.my_navigation("c_start")
        ArmMovement.end(self.arm)
