import time

from ..param import NavMovement, ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.GrabAppleTree import GrabAppleTree
from ..util.Logger import Logger
from ..util.Singleton import singleton


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
            self.robot.set_start_led(True)
            task_method()
            self.robot.set_start_led(False)
            self.logger.info(f"任务B-任务{i}完成")
        else:
            self.logger.error(f"任务B-任务{i}不存在")

    def task_1(self):
        # 去黄站台2识别水果
        NavMovement.start_to_yellow_station_2(self.move)
        self.move.rotation_correction()
        ArmMovement.identify_station_fruit(self.arm, Direction.LEFT)
        yellow_station = None
        while yellow_station is None:
            yellow_station = self.vision.find_fruit()
            time.sleep(1)
        ArmMovement.motion(self.arm)

        # 去红站台1抓取水果
        NavMovement.yellow_station_2_to_red_station_1(self.move)
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)

        ArmMovement.identify_station_fruit(self.arm, Direction.LEFT)
        red_station = None
        while red_station is None:
            red_station = self.vision.find_fruit([FruitType(yellow_station.class_id)])
            time.sleep(1)
        move_dis = Math.pixel_to_horizontal_distance_x_centered(red_station.box.get_rectangle_center().x - 320, 0.15)
        self.move.line(move_dis - 0.04)

        ArmMovement.grab_fruit_from_station(self.arm)

        ArmMovement.motion(self.arm)

    def corrective_form_front(self, dis):
        from_wall = self.sensor.get_distance_from_wall(Direction.FRONT)
        wall_dis = from_wall - dis
        while abs(wall_dis) > 0.02:
            from_wall = self.sensor.get_distance_from_wall(Direction.FRONT)
            wall_dis = from_wall - dis
            self.move.line(wall_dis)

    def task_2(self):
        # 去黄平台1抓取果篮
        NavMovement.start_to_yellow_station_1(self.move)
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        count = 0
        # 去葡萄园抓取3个地上的水果放到框子里
        NavMovement.yellow_station_1_to_vineyard(self.move)
        ArmMovement.identify_ground_fruit(self.arm)
        coordinate_point = Math.get_target_coordinate(NavMovement.VINEYARD_1, 2)
        self.move.navigation([coordinate_point], 0.1, False)
        while self.move.get_status() :
            fruit = self.vision.find_fruit()
            if fruit:
                self.move.stop_navigation()
                fruit = self.vision.find_fruit()
                if fruit:
                    center = fruit.box.get_rectangle_center()

                    distance = 0.41
                    if fruit.distance != -1:
                        distance = fruit.distance

                    # 先移动到水果前面，使其在夹爪下方
                    camera_to_claws = 0.05
                    move_dis = Math.pixel_to_distance_from_center(center.y, distance)
                    self.move.line(move_dis + camera_to_claws)

                    # 计算水果的左右偏移
                    x_dis = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
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
                    self.arm.rotate(180 + rotary_angle)

                    # 抓取
                    self.arm.lift(31)
                    if "grape" in fruit.class_id:
                        ArmMovement.close_gripper_grape(self.arm)
                    else:
                        ArmMovement.close_gripper_apple(self.arm)
                    self.arm.lift(0)

                    ArmMovement.put_fruit_to_basket(self.arm, 1)

                    count += 1
                    if count >= 3:
                        print("yer~")
                        break

                    ArmMovement.identify_ground_fruit(self.arm)

                angle_from_wall = self.sensor.get_angle_from_wall(Direction.LEFT)
                if abs(angle_from_wall) < 10:
                    self.sensor.init_odom_yaw(coordinate_point.yaw - angle_from_wall)

                self.move.navigation([coordinate_point], 0.05, False)
        ArmMovement.motion(self.arm)

        # 前往黄平台3放置果篮
        NavMovement.vineyard_1_to_yellow_station_3(self.move)
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

    def task_3(self):
        # 去果树1抓一颗苹果
        NavMovement.start_to_tree1(self.move)
        grab_apple_tree = GrabAppleTree(self.node, Direction.LEFT)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE]
        grab_apple_tree.grab_apple_from_tree()

        # 如果正面没有，去侧面看一下
        if grab_apple_tree.has_apple():
            self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_TREE_1_2])
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()

    def grab_grape(self, grape: IdentifyResult, direction: Direction):
        center = grape.box.get_rectangle_center()

        distance = 0.25
        if grape.distance != -1:
            distance = grape.distance
        x_dis = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)

        if direction == Direction.LEFT:
            self.move.line(x_dis)
        elif direction == Direction.RIGHT:
            self.move.line(-x_dis)

        ArmMovement.grab_grape_on_wall(self.arm, direction, center.y > 280)

    def task_4(self):
        # 前往葡萄园
        NavMovement.start_to_vineyard_1(self.move)

        # 寻找葡萄并抓取
        target = [FruitType.PURPLE_GRAPE]
        ArmMovement.identify_grape(self.arm, Direction.RIGHT)
        coordinate_point = Math.get_target_coordinate(NavMovement.VINEYARD_1, 2)
        self.move.navigation([coordinate_point], 0.1, False)
        while self.move.get_status():
            grape = self.vision.find_fruit(target)
            if grape:
                self.move.stop_navigation()
                self.grab_grape(grape, Direction.RIGHT)
        ArmMovement.motion(self.arm)

    def grab_basket_from_station(self, direction: Direction):
        ArmMovement.station_basket_top(self.arm, direction)
        ArmMovement.open_gripper(self.arm)
        self.arm.nod_servo(0)
        distance_from_wall = self.sensor.get_distance_from_wall(direction)
        self.arm.telescopic_servo((distance_from_wall - 0.25) * 100)

        self.arm.lift(9.5)
        ArmMovement.close_gripper_basket(self.arm)
        self.arm.lift(0, is_block=False)
        self.arm.rotate(0, is_block=False)
        self.arm.wait_finish()

    def task_5(self):
        # 去抓第1个框子
        NavMovement.start_to_yellow_station_1(self.move)
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去抓第2个框子
        self.move.navigation([NavMovement.YELLOW_STATION_2])
        self.move.rotation_correction()
        self.corrective_form_front(1.11)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去抓第3个框子
        self.move.navigation([NavMovement.CORRECTIVE_POINT_2, NavMovement.YELLOW_STATION_3])
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 3)
        ArmMovement.motion(self.arm)

        # 去放第1个框子
        self.move.navigation([NavMovement.RED_STATION_3])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

        # 去放第2个框子
        self.move.navigation([NavMovement.RED_STATION_2])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 2)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

        # 去放第3个框子
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.RED_STATION_1])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 3)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

    def task_6(self):
        # 去黄站台1抓一个篮子
        NavMovement.start_to_yellow_station_1(self.move)
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去苹果区抓一个苹果，放到篮子里
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_TREE_1_1])
        self.move.rotation_correction()
        self.corrective_form_front(0.42)
        ArmMovement.grab_apple_on_tree(self.arm, Direction.LEFT, 0, False)
        ArmMovement.put_fruit_to_basket(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去葡萄区抓一个葡萄，放到篮子里
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.CORRECTIVE_POINT_2, NavMovement.VINEYARD_1, NavMovement.VINEYARD_2])
        ArmMovement.identify_grape(self.arm, Direction.RIGHT)
        ArmMovement.grab_grape_on_wall(self.arm, Direction.RIGHT)
        ArmMovement.put_fruit_to_basket(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 将果篮放在红站台1
        self.move.navigation([NavMovement.VINEYARD_1, NavMovement.CORRECTIVE_POINT_2, NavMovement.RED_STATION_1])
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)

        # 回到起始区
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.START_POINT])

    def task_7(self):
        # 去黄站台2抓框子
        NavMovement.start_to_yellow_station_2(self.move)
        self.move.rotation_correction()
        self.corrective_form_front(1.11)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去红站台1抓框子
        self.move.navigation([NavMovement.RED_STATION_1])
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去红站台2抓框子
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.RED_STATION_2])
        self.move.rotation_correction()
        self.corrective_form_front(1.11)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 3)
        ArmMovement.motion(self.arm)

        grab_apple_tree = GrabAppleTree(self.node, Direction.LEFT)
        grab_apple_tree.basket_1 = [FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.RED_APPLE]
        # grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_GRAPE]
        # 去果树2抓6颗水果，分别为红绿、绿黄、黄紫
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_TREE_1_1])
        self.move.rotation_correction()
        grab_apple_tree.grab_apple_from_tree()

        # 如果还有水果没抓，去侧面看看
        if grab_apple_tree.has_apple():
            self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_TREE_1_2])
            self.move.rotation_correction()
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()

        # 回起始区
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.START_POINT])

    def find_grape_and_grab(self, coordinate_point, direction: Direction, basket_1, basket_2, basket_3):
        ArmMovement.identify_grape(self.arm, direction)
        self.move.navigation([coordinate_point], 0.1, False)
        while self.move.get_status():
            grape = self.vision.find_fruit(basket_1 + basket_2 + basket_3)
            if grape:
                self.move.stop_navigation()
                grape = self.vision.find_fruit(basket_1 + basket_2 + basket_3)
                if grape:
                    fruit_type = FruitType(grape.class_id)
                    if fruit_type in basket_1:
                        basket_1.remove(fruit_type)
                        self.grab_grape(grape, direction)
                        ArmMovement.put_fruit_to_basket(self.arm, 1)
                    elif fruit_type in basket_2:
                        basket_2.remove(fruit_type)
                        self.grab_grape(grape, direction)
                        ArmMovement.put_fruit_to_basket(self.arm, 2)
                    elif fruit_type in basket_3:
                        basket_3.remove(fruit_type)
                        self.grab_grape(grape, direction)
                        ArmMovement.put_fruit_to_basket(self.arm, 3)

                    ArmMovement.identify_grape(self.arm, direction)

                angle_from_wall = self.sensor.get_angle_from_wall(Direction.LEFT)
                if abs(angle_from_wall) < 10:
                    self.sensor.init_odom_yaw(coordinate_point.yaw - angle_from_wall)

                self.move.navigation([coordinate_point], 0.1, False)
        ArmMovement.motion(self.arm)

    def task_8(self):
        # 去黄站台1抓框子
        NavMovement.start_to_yellow_station_1(self.move)
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去黄站台2抓框子
        self.move.navigation([NavMovement.YELLOW_STATION_2])
        self.move.rotation_correction()
        self.corrective_form_front(1.11)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去黄站台3抓框子
        self.move.navigation([NavMovement.CORRECTIVE_POINT_2, NavMovement.YELLOW_STATION_3])
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 3)
        ArmMovement.motion(self.arm)

        basket_1 = [FruitType.GREEN_GRAPE, FruitType.YELLOW_GRAPE]
        basket_2 = [FruitType.YELLOW_GRAPE, FruitType.PURPLE_GRAPE]
        basket_3 = [FruitType.PURPLE_GRAPE, FruitType.GREEN_GRAPE]
        # 前往葡萄区抓取葡萄
        self.move.navigation([NavMovement.VINEYARD_1])
        coordinate_point = Math.get_target_coordinate(NavMovement.VINEYARD_1, 2)
        self.find_grape_and_grab(coordinate_point, Direction.RIGHT, basket_1, basket_2, basket_3)

        #返回起始区
        self.move.navigation([NavMovement.VINEYARD_1, NavMovement.CORRECTIVE_POINT_2, NavMovement.CORRECTIVE_POINT_1, NavMovement.START_POINT])

    def task_9(self):
        # 去果树1抓一颗苹果
        self.move.navigation([NavMovement.START_POINT, NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_RANDOM_TREE_1_1])
        grab_apple_tree = GrabAppleTree(self.node, Direction.LEFT)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE]
        grab_apple_tree.grab_apple_from_tree()

        # 如果正面没有，去侧面看一下
        if grab_apple_tree.has_apple():
            self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_RANDOM_TREE_1_2])
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()

    def task_10(self):
        # 去黄站台2抓框子
        NavMovement.start_to_yellow_station_2(self.move)
        self.move.rotation_correction()
        self.corrective_form_front(1.11)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 1)
        ArmMovement.motion(self.arm)

        # 去红站台1抓框子
        self.move.navigation([NavMovement.RED_STATION_1])
        self.move.rotation_correction()
        self.sensor.ping_revise(0.89)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 2)
        ArmMovement.motion(self.arm)

        # 去红站台2抓框子
        self.move.navigation([NavMovement.RED_STATION_2])
        self.move.rotation_correction()
        self.corrective_form_front(1.11)
        self.grab_basket_from_station(Direction.LEFT)
        ArmMovement.put_basket_to_robot_b(self.arm, 3)
        ArmMovement.motion(self.arm)

        grab_apple_tree = GrabAppleTree(self.node, Direction.LEFT)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.RED_APPLE, FruitType.PURPLE_GRAPE]
        grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_GRAPE]
        # 去果树2抓6颗水果，分别为红绿、绿黄、黄紫
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_RANDOM_TREE_1_1])
        self.move.rotation_correction()
        grab_apple_tree.grab_apple_from_tree()

        # 如果还有水果没抓，去侧面看看
        if grab_apple_tree.has_apple():
            self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.IDENTIFY_RANDOM_TREE_1_2])
            self.move.rotation_correction()
            grab_apple_tree.direction = Direction.RIGHT
            grab_apple_tree.grab_apple_from_tree()

        # 回起始区
        self.move.navigation([NavMovement.CORRECTIVE_POINT_1, NavMovement.START_POINT])

