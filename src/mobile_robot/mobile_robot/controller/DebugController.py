from ..dao.RobotDataDao import RobotDataDao
from ..param import ArmMovement
from ..popo.OmsGoal import OmsGoal
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..util.Logger import Logger


class DebugController:
    def __init__(self, node):
        self.node = node
        self.logger = Logger()
        self.arm = ArmService(node)
        self.robot = RobotService(node)
        self.robot_data = RobotDataDao(node)
        self.move = MoveService(node)

    def run(self):
        self.robot.with_robot_connect()

        while True:
            print("0. 复位")
            print("1. 夹爪旋转控制")
            print("2. 夹爪点头控制")
            print("3. 夹爪伸缩控制")
            print("4. 夹爪夹合控制")
            print("5. 机械臂升降控制")
            print("6. 机械臂旋转控制")
            print("7. 直线运动")
            print("8. 旋转运动")
            print("9. 已有动作")
            print("q. 退出")

            choice = input("请输入指令，或输入q退出:")
            if choice == "0":
                self.arm.back_origin()
                self.arm.plan_once(OmsGoal(servo_rotary=0, servo_nod=0, servo_telescopic=0, servo_gripper=20))
                print("已复位")
            if choice == "1":
                self.servo_rotary()
            elif choice == "2":
                self.servo_nod()
            elif choice == "3":
                self.servo_telescopic()
            elif choice == "4":
                self.servo_gripper()
            elif choice == "5":
                self.arm_lift()
            elif choice == "6":
                self.arm_rotate()
            elif choice == "7":
                self.line()
            elif choice == "8":
                self.rotate()
            elif choice == "9":
                self.arm_movement()
            elif choice == "q":
                exit(0)
            else:
                print("无效的指令，请重新输入")

    def arm_movement(self):
        while True:
            # 动态获取任务，拒绝重复代码
            task = input("输入动作名称, q 退出: ")
            if task == "q":
                return

            task_method = getattr(ArmMovement, f"{task}")
            if callable(task_method):
                self.logger.info(f"开始执行{task}")
                task_method()
                self.logger.info(f"动作{task}完成")
            else:
                self.logger.error(f"{task}不存在")

    def servo_rotary(self):
        while True:
            i = input("输入夹爪旋转角度，或输入q退出: ")
            if i == "q":
                break
            try:
                self.arm.servo_rotary(float(i))
            except ValueError:
                print("输入无效，请输入一个数字。")

    def servo_nod(self):
        while True:
            i = input("输入夹爪抬头角度，或输入q退出: ")
            if i == "q":
                break
            try:
                self.arm.servo_nod(float(i))
            except ValueError:
                print("输入无效，请输入一个数字。")

    def servo_telescopic(self):
        while True:
            i = input("输入夹爪伸缩距离，或输入q退出: ")
            if i == "q":
                break
            try:
                self.arm.servo_telescopic(float(i))
            except ValueError:
                print("输入无效，请输入一个数字。")

    def servo_gripper(self):
        while True:
            i = input("输入夹爪张开距离，或输入q退出: ")
            if i == "q":
                break
            try:
                self.arm.servo_gripper(float(i))
            except ValueError:
                print("输入无效，请输入一个数字。")

    def arm_lift(self):
        while True:
            i = input("输入升降高度，或输入q退出: ")
            if i == "q":
                break
            try:
                self.arm.lift(float(i), 40, True)
            except ValueError:
                print("输入无效，请输入一个数字。")

    def arm_rotate(self):
        while True:
            i = input("输入旋转角度，或输入q退出: ")
            if i == "q":
                break
            try:
                self.arm.rotate(float(i), 70, True)
            except ValueError:
                print("输入无效，请输入一个数字。")

    def line(self):
        while True:
            i = input("输入行进距离，或输入q退出: ")
            if i == "q":
                break
            try:
                self.move.line(float(i))
            except ValueError:
                print("输入无效，请输入一个数字。")

    def rotate(self):
        while True:
            i = input("输入旋转角度，或输入q退出: ")
            if i == "q":
                break
            try:
                self.move.rotate(float(i))
            except ValueError:
                print("输入无效，请输入一个数字。")
