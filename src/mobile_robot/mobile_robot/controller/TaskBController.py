import time

from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.GrabAppleTree import GrabAppleTree
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.GrabGroundFruit import GrabGroundFruit
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

        self.robot.set_start_led(True)
        self.arm.back_origin()

        try:
            i = int(i)
        except Exception:
            self.logger.error("输入的不是数字, 默认从1开始.")
            i = 1

        for i in range(i, 11):
            # 动态获取任务，拒绝重复代码
            task_method = getattr(self, f"task_{i}")
            if callable(task_method):
                self.logger.info(f"等待执行任务B-{i}")
                # self.sensor.correction("c_4")
                self.sensor.init_odom_all(NavigationPoint(2.432, 2.452, 180))
                button = self.robot.with_start_button()
                self.logger.info(f"开始执行任务B-{i}")
                self.arm.plan_list(ArmMovement.motion())
                start_time = time.time()
                task_method(button)
                self.robot.set_start_led(False)
                end_time = time.time()
                use_time = end_time - start_time
                self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")
                self.logger.info(f"任务B-{i}完成")
            else:
                self.logger.error(f"任务B-{i}不存在")

    def task_1(self, _):
        # 去黄站台2识别水果
        self.logger.info("导航前往黄站台2")
        self.vision.set_other_fruit_weight()
        Station.YELLOW_2.nav_to_station(self.node)
        self.logger.info("开始识别水果")
        ArmMovement.identify_station_fruit(self.arm, Direction.RIGHT)
        s_y_result = None
        while s_y_result is None:
            s_y_result = self.vision.find_fruit(FruitType.all(), True)
            time.sleep(1)
        self.logger.info("识别成功，准备移动至红站台1")
        self.arm.plan_list(ArmMovement.motion())

        # 去红站台1抓取水果
        direction = Direction.LEFT
        Station.RED_1.nav_to_station(self.node)
        Station.RED_1.correction(self.node)
        self.logger.info("开始识别水果")
        ArmMovement.identify_station_fruit(self.arm, direction)
        red_station = None
        while red_station is None:
            red_station = self.vision.find_fruit([FruitType(s_y_result.class_id)], True)
            time.sleep(1)
        move_dis = Math.pixel_to_horizontal_distance_x_centered(red_station.box.get_rectangle_center().x - 320, 0.2)
        self.logger.info(f"寻找到横向 {move_dis} 米的距离处有一个 {red_station.class_id} 水果")
        # 往左抓 -0.07 往右抓 +0.07
        self.move.line(move_dis - 0.07)

        self.logger.info("准备抓取水果")
        ArmMovement.grab_fruit_from_station(self.arm, direction, True)

        ArmMovement.motion_apple(self.arm)

    def task_2(self, _):
        self.vision.set_other_fruit_weight()

        # 去黄平台1抓取果篮
        self.logger.info("导航前往黄站台1")
        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2) + ArmMovement.motion(), block=False)

        ground_fruit = GrabGroundFruit(self.node)
        ground_fruit.goal_num = 3
        def grab_fruits(_path, target_waypoint):
            """前往指定点抓水果，返回更新后的 count"""
            if _path:
                for r in _path:
                    self.move.my_navigation(r)

                # 当走到头时会返回False，进入下一个路径
            return ground_fruit.patrol_line(target_waypoint)

        # 依次尝试的路线
        plans = [
            (["c_2"], "c_3"),
            (["v_9"], "v_10"),
            (["c_4", "v_2"], "c_5")
        ]

        for path, point in plans:
            self.logger.info(f"尝试扫描到 {point} 抓水果")
            if grab_fruits(path, point):
                break
            self.arm.plan_list(ArmMovement.motion(), block=False)
            self.move.my_navigation(point, start_name=point)

        self.logger.info("抓取完毕，前往放置果篮")
        Station.YELLOW_3.nav_and_put(self.node, 2)
        self.arm.plan_list(ArmMovement.motion())

    def task_3(self, button):
        # 去果树1抓一颗苹果
        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE]

        if button:
            self.logger.info("检测到按键，开始前往 树1前")
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

    def task_4(self, _):
        # 寻找葡萄并抓取
        self.move.my_navigation("v_4")
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE]
        grab_grape_wall.grab_grape_from_wall()

    def task_5(self, _):
        # 去抓第1个框子
        self.logger.info("导航前往黄站台2")
        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        self.arm.plan_list(ArmMovement.motion())

        # 去抓第2个框子
        self.logger.info("导航前往黄站台1")
        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2) + ArmMovement.motion(), block=False)

        # 去抓第3个框子
        self.logger.info("导航前往黄站台3")
        Station.YELLOW_3.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion(), block=False)

        # 去放第1个框子
        self.logger.info("导航前往红站台3")
        Station.RED_3.nav_and_put(self.node, 3)
        self.arm.plan_list(ArmMovement.motion())

        # 去放第2个框子
        self.logger.info("导航前往红站台2")
        Station.RED_2.nav_and_put(self.node, 2)
        self.arm.plan_list(ArmMovement.motion())

        # 去放第3个框子
        self.logger.info("导航前往红站台1")
        Station.RED_1.nav_and_put(self.node, 1)
        self.arm.plan_list(ArmMovement.motion())

    def task_6(self, _):
        # 去黄站台1抓一个篮子
        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2) + ArmMovement.motion(), block=False)

        # 去葡萄区抓一个葡萄，放到篮子里
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.RIGHT
        self.move.my_navigation("v_1")
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.basket_2 = [FruitType.PURPLE_GRAPE]
        grab_grape_wall.grab_grape_from_wall()

        # 去苹果区抓一个苹果，放到篮子里
        self.move.my_navigation("c_7")
        self.move.my_navigation("t_1_g")
        ArmMovement.grab_apple_on_tree(self.arm, Direction.RIGHT, 0, False, 0)
        ArmMovement.put_fruit_to_basket(self.arm, 2)
        self.arm.plan_list(ArmMovement.motion())

        # 将果篮放在红站台1
        Station.RED_1.nav_and_put(self.node, 2)
        self.arm.plan_list(ArmMovement.motion())

        # 回到起始区
        self.move.my_navigation("start")
        self.arm.servo_nod(-60)

    def task_7(self, _):
        # 去黄站台2抓框子
        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        self.arm.plan_list(ArmMovement.motion(), block=False)

        # 去红站台1抓框子
        Station.RED_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2))
        self.arm.plan_list(ArmMovement.motion())

        # 去红站台2抓框子
        self.move.my_navigation("c_7")
        Station.RED_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion(), block=False)

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
        self.move.my_navigation("start")
        self.arm.servo_nod(-60)

    def task_8(self, _):
        # 去黄站台2抓框子
        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        self.arm.plan_list(ArmMovement.motion())

        # 去黄站台1抓框子
        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2) + ArmMovement.motion(), block=False)

        # 去黄站台3抓框子
        Station.YELLOW_3.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion(), block=False)

        # 前往葡萄区抓取葡萄
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.basket_1 = [FruitType.GREEN_GRAPE, FruitType.YELLOW_GRAPE]
        grab_grape_wall.basket_2 = [FruitType.YELLOW_GRAPE, FruitType.PURPLE_GRAPE]
        grab_grape_wall.basket_3 = [FruitType.PURPLE_GRAPE, FruitType.GREEN_GRAPE]

        self.move.my_navigation("c_5")
        path = ["v_7", "v_8"]
        for i in path:
            self.move.my_navigation(i)
            grab_grape_wall.grab_grape_from_wall()
            if not grab_grape_wall.has_grape():
                break

        # 返回起始区
        self.move.my_navigation("start")
        self.arm.servo_nod(-60)

    def task_9(self, button):
        # 去会动的 果树1 抓一颗苹果
        # 目前跟第 3 题一样
        self.task_3(button)

    def task_10(self, _):
        # 去黄站台2抓框子
        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        self.arm.plan_list(ArmMovement.motion(), block=False)

        # 去红站台1抓框子
        Station.RED_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2))
        self.arm.plan_list(ArmMovement.motion())

        # 去红站台2抓框子
        self.move.my_navigation("c_7")
        Station.RED_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion(), block=False)

        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]
        # # 去果树3抓6颗水果，分别为红绿、绿黄、黄紫
        self.move.my_navigation("t_3_r", speed=0.4)
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
        self.move.my_navigation("start")
        self.arm.servo_nod(-60)
