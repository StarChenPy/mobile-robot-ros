import rclpy

from ..param import ArmMovement, NavigationPath
from ..popo.Direction import Direction
from ..popo.FruitHeight import FruitHeight
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Util
from ..util.Singleton import singleton


@singleton
class BModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__robot = RobotService(node)
        self.__arm = ArmService(node)
        self.__move = MoveService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def task(self, select: int):
        for i in range(select, 16):
            self.__robot.with_start_button()
            self.__sensor.reset_odom()

            self.task_select(i)

            self.__robot.set_start_led(False)

    def task_select(self, select: int):
        match select:
            case 1: self.task1()
            case 2: self.task2()
            case 3: self.task3()
            case 4: self.task4()
            case 5: self.task5()
            case 6: self.task6()
            case 7: self.task7()
            case 8: self.task8()
            case 9: self.task9()
            case 10: self.task10()
            case 11: self.task11()
            case 12: self.task12()
            case 13: self.task13()
            case 14: self.task14()
            case 15: self.task15()
            case _: exit(0)

    def task1(self):
        # 直线1米
        self.__sensor.init_odom_all(NavigationPoint(0, 0, 0))
        self.__move.navigation([NavigationPoint(1, 0, 0)])

    def task2(self):
        # 旋转360度
        self.__move.rotate(361)

    def task3(self):
        # 抓水果
        self.__arm.grab_fruit(FruitHeight.TALL.value, Direction.RIGHT)

    def task4(self):
        # 果仓1到起始区
        self.__sensor.init_odom_all(NavigationPoint(0.56, -1.12, 90))
        self.__move.navigation(NavigationPath.B_MODULE_4)

    def task5(self):
        # 起始区到果仓1
        self.__move.navigation(NavigationPath.B_MODULE_5)

    def task6(self):
        # 采摘1到起始区
        self.__sensor.init_odom_all(NavigationPath.ORCHARD_1_POINT)
        self.__move.navigation(NavigationPath.B_MODULE_6)

    def task7(self):
        # 起始区到采摘1
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

    def task8(self):
        # 起始区到采摘1抓高水果
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

        self.__arm.grab_fruit(FruitHeight.TALL.value, Direction.RIGHT)

    def task9(self):
        # 起始区到采摘1抓中水果
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

        self.__arm.grab_fruit(FruitHeight.MIDDLE.value, Direction.RIGHT)

    def task10(self):
        # 起始区到采摘1抓低水果
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

        self.__arm.grab_fruit(FruitHeight.LOW.value, Direction.RIGHT)

    def task11(self):
        # 起始区到果仓一号放水果
        self.__move.navigation(NavigationPath.B_MODULE_11)

        self.__arm.control(ArmMovement.READY_PULL_WAREHOUSE)
        self.__arm.control(ArmMovement.PULL_WAREHOUSE)
        self.__arm.control(ArmMovement.MOVING)

    def task12(self):
        # 起始区到果园摘水果然后去果仓1号放水果
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

        self.__arm.grab_fruit(FruitHeight.TALL.value, Direction.RIGHT)

        self.__move.navigation(NavigationPath.B_MODULE_12)

        self.__arm.control(ArmMovement.READY_PULL_WAREHOUSE)
        self.__arm.control(ArmMovement.PULL_WAREHOUSE)
        self.__arm.control(ArmMovement.MOVING)

    def task13(self):
        # 起始区到果仓识别一个水果
        self.__move.navigation(NavigationPath.B_MODULE_5)

        self.__arm.control(ArmMovement.RECOGNITION_WAREHOUSE)
        for _ in range(10):
            print(self.__vision.get_onnx_identify_result())
        self.__arm.control(ArmMovement.MOVING)

    def task14(self):
        # 起始区到果园识别一个水果
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

        ArmMovement.recognition_orchard(self.__arm, Direction.RIGHT)

        for _ in range(10):
            print(self.__vision.get_onnx_identify_result())
        self.__arm.control(ArmMovement.MOVING)

    def task15(self):
        # 起始区到果园识别一个水果的高低
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

        ArmMovement.recognition_orchard(self.__arm, Direction.RIGHT)

        for _ in range(10):
            result = self.__vision.get_onnx_identify_result()
            for e in result:
                depth_data = self.__vision.get_depth_data(e.box.get_rectangle_center())
                match Util.get_fruit_height(depth_data):
                    case FruitHeight.TALL:
                        print("高水果")
                    case FruitHeight.MIDDLE:
                        print("中水果")
                    case FruitHeight.LOW:
                        print("低水果")
        self.__arm.control(ArmMovement.MOVING)
