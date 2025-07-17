import threading
import time

from ..dao.RobotDataDao import RobotDataDao
from ..popo.ArmMovement import ArmMovement
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService
from ..param import ArmMovement as Movement
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SowerServoService import SowerServoService

# 开合24 伸缩6 升降16


toggle = False


def toggle_func():
    global toggle
    while True:
        if toggle:
            input()
            toggle = False


class PuchipuchiController:
    def __init__(self, node):
        self.node = node
        self.arm = ArmService(node)
        self.robot = RobotService(node)
        self.sower = SowerServoService(node)
        self.robot_data = RobotDataDao(node)
        self.move = MoveService(node)

        self.prev_state = False
        self.rising_edge_count = 0

    def run(self):
        self.robot.with_robot_connect()

        while True:
            print("请输入指令，抓取演示请使用2-3，播种请使用4-2")
            print("0. 复位")
            print("1. 抓果-基本控制")
            print("2. 抓果-功能演示")
            print("3. 播种-基本控制")
            print("4. 播种-功能演示")
            print("5. 移动控制")
            print("q. 退出")

            choice = input()
            if choice == "0":
                self.arm.back_origin()
                print("已复位")
            elif choice == "1":
                self.grab_basic_control()
            elif choice == "2":
                self.grab_func_control()
            elif choice == "3":
                self.sower_basic_control()
            elif choice == "4":
                self.sower_func_control()
            elif choice == "5":
                self.move_control()
            elif choice == "q":
                exit(0)

    def grab_basic_control(self):
        while True:
            print("请输入指令")
            print("1. 夹爪旋转控制")
            print("2. 夹爪点头控制")
            print("3. 夹爪伸缩控制")
            print("4. 夹爪夹合控制")
            print("5. 机械臂升降控制")
            print("6. 机械臂旋转控制")
            print("q. 退出")

            choice = input()
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
            elif choice == "q":
                break

    def servo_rotary(self):
        while True:
            i = input("输入夹爪旋转角度，或输入q退出: ")
            if i == "q":
                break
            self.arm.rotary_servo(float(i))

    def servo_nod(self):
        while True:
            i = input("输入夹爪抬头角度，或输入q退出: ")
            if i == "q":
                break
            self.arm.nod_servo(float(i))

    def servo_telescopic(self):
        while True:
            i = input("输入夹爪伸缩距离，或输入q退出: ")
            if i == "q":
                break
            self.arm.telescopic_servo(float(i))

    def servo_gripper(self):
        while True:
            i = input("输入夹爪张开距离，或输入q退出: ")
            if i == "q":
                break
            self.arm.gripper_servo(float(i))

    def arm_lift(self):
        while True:
            i = input("输入升降高度，或输入q退出: ")
            if i == "q":
                break
            self.arm.lift(float(i), 40, True)

    def arm_rotate(self):
        while True:
            i = input("输入旋转角度，或输入q退出: ")
            if i == "q":
                break
            self.arm.rotate(float(i), 40, True)

    def grab_func_control(self):
        while True:
            print("请输入指令")
            print("1. 抓取水果")
            print("2. 抓取果篮")
            print("3. 抓三个篮子然后抓水果放到框子中")
            print("q. 退出")

            choice = input()
            if choice == "1":
                self.grab_fruit()
            if choice == "2":
                self.grab_basket()
            if choice == "3":
                self.grab_basket_and_fruit()
            elif choice == "q":
                break

    def grab_fruit(self):
        self.arm.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, -90, 12, 20)))
        self.arm.control(ArmMovement(MotorMovement(180, 32)))
        time.sleep(0.5)
        self.arm.control(ArmMovement(servo=ServoMotor(0, -90, 12, 7)))
        self.arm.control(ArmMovement(MotorMovement(0, 18)))

    def grab_basket(self):
        self.arm.control(ArmMovement(MotorMovement(180, 8), ServoMotor(0, -90, 12, 25)))
        self.arm.control(ArmMovement(MotorMovement(180, 21)))
        self.arm.control(ArmMovement(servo=ServoMotor(0, -90, 12, 19)))
        self.arm.control(ArmMovement(MotorMovement(180, 8), ServoMotor(0, 0, 12, 19)))
        self.arm.control(ArmMovement(MotorMovement(0, 8)))

    def grab_basket_and_fruit(self):
        # 抓取三个篮子
        for i in range(1, 4):
            self.arm.control(ArmMovement(MotorMovement(180, 8), ServoMotor(0, -90, 12, 25)))
            self.arm.control(ArmMovement(MotorMovement(180, 21)))
            self.arm.control(ArmMovement(servo=ServoMotor(0, -90, 12, 19)))
            self.arm.control(ArmMovement(MotorMovement(180, 8), ServoMotor(0, 0, 12, 19)))
            self.arm.control(ArmMovement(MotorMovement(0, 8)))

            Movement.put_basket_to_robot(self.arm, 4 - i)

        # 抓取三个水果
        for i in range(1, 4):
            self.arm.control(ArmMovement(MotorMovement(180, 2), ServoMotor(0, -90, 12, 20)))
            self.arm.control(ArmMovement(MotorMovement(180, 32)))
            time.sleep(0.5)
            self.arm.control(ArmMovement(servo=ServoMotor(0, -90, 12, 7)))
            self.arm.control(ArmMovement(MotorMovement(180, 14), ServoMotor(0, 0, 12, 7)))
            self.arm.control(ArmMovement(MotorMovement(0, 14), ServoMotor(0, 0, 0, 7)))

            Movement.put_fruit_into_basket(self.arm, i, 7)

        self.arm.control(Movement.MOVING)

    def sower_basic_control(self):
        while True:
            print("请输入指令")
            print("1. 出口切换控制")
            print("2. 出料筒控制")
            print("3. 播种伸缩控制")
            print("4. 旋钮旋转控制")
            print("5. 机械臂升降控制")
            print("6. 机械臂旋转控制")
            print("q. 退出")

            choice = input()
            if choice == "1":
                self.exit_toggle_servo()
            elif choice == "2":
                self.servo_sower()
            elif choice == "3":
                self.servo_sower_telescopic()
            elif choice == "4":
                self.servo_knob_rotate()
            elif choice == "5":
                self.arm_lift()
            elif choice == "6":
                self.arm_rotate()
            elif choice == "q":
                break

    def exit_toggle_servo(self):
        threading.Thread(target=toggle_func, daemon=True).start()
        while True:
            i = input("输入c顺时针旋转，输入w逆时针旋转，或输入q停止并退出: ")
            if i == "q":
                self.sower.servo_exit_toggle(False, False)
                break
            elif i == "c":
                self.sower.servo_exit_toggle(True)
            elif i == "w":
                self.sower.servo_exit_toggle(False)

            global toggle
            toggle = True
            while True:
                if not toggle:
                    self.sower.servo_exit_toggle(False, False)
                    break

                current_state = self.robot_data.get_robot_data().titan_limit_sw[2].lim_h

                # 检测上升沿
                if not self.prev_state and current_state:
                    if i == "c":
                        self.rising_edge_count += 1
                    elif i == "w":
                        self.rising_edge_count -= 1
                    print(f"当前计数: {self.rising_edge_count}")

                self.prev_state = current_state

    def servo_sower(self):
        while True:
            i = input("按任意键播种一次，或输入q退出: ")
            if i == "q":
                self.sower.servo_sower(False)
                break
            self.sower.servo_sower()

    def servo_sower_telescopic(self):
        while True:
            i = input("输入伸缩距离，或输入q退出: ")
            self.sower.telescopic_servo(0, enable=False)
            if i == "q":
                break
            self.sower.telescopic_servo(float(i))

    def servo_knob_rotate(self):
        while True:
            i = input("输入开启 t 或是关闭 f，或输入q退出: ")
            if i == "q":
                self.sower.servo_knob_rotate(False, False)
                break
            elif i == "t":
                self.sower.servo_knob_rotate(True)
            elif i == "f":
                self.sower.servo_knob_rotate(False)

    def sower_func_control(self):
        while True:
            print("请输入指令")
            print("1. 装填种子")
            print("2. 边吃边拉")
            print("q. 退出")

            choice = input()
            if choice == "1":
                self.fill_the_seeds()
            elif choice == "2":
                self.pull_while_eating()
            elif choice == "q":
                break

    def fill_the_seeds(self):
        while True:
            i = input("输入 a 或 b 选择将种子放入哪个通道，或输入q退出: ")
            if i == "q":
                break
            elif i == "a":
                if self.rising_edge_count != 0:
                    self.sower.servo_exit_toggle(False)

                    while True:
                        current_state = self.robot_data.get_robot_data().titan_limit_sw[2].lim_h
                        # 检测上升沿
                        if not self.prev_state and current_state:
                            self.rising_edge_count -= 1
                        self.prev_state = current_state

                        if self.rising_edge_count == 0:
                            self.sower.servo_exit_toggle(True, False)
                            break
            elif i == "b":
                if self.rising_edge_count != 5:
                    self.sower.servo_exit_toggle(True)

                    while True:
                        current_state = self.robot_data.get_robot_data().titan_limit_sw[2].lim_h
                        # 检测上升沿
                        if not self.prev_state and current_state:
                            self.rising_edge_count += 1
                        self.prev_state = current_state

                        if self.rising_edge_count == 5:
                            self.sower.servo_exit_toggle(False, False)
                            break

            self.sower.servo_knob_rotate(True)
            time.sleep(5)
            self.sower.servo_knob_rotate(False)

    def pull_while_eating(self):
        flag = True

        while True:
            self.robot.with_start_button()

            self.sower.servo_sower()
            self.sower.servo_knob_rotate(True)
            time.sleep(5)
            self.sower.servo_knob_rotate(False)

            self.sower.servo_exit_toggle(False)

            while True:
                current_state = self.robot_data.get_robot_data().titan_limit_sw[2].lim_h
                # 检测上升沿
                if not self.prev_state and current_state:
                    self.rising_edge_count += 1
                self.prev_state = current_state
                flag_num = 16 if flag else 15
                if self.rising_edge_count == flag_num:
                    flag = not flag
                    self.sower.servo_exit_toggle(True, False)
                    self.rising_edge_count = 0
                    break

    def move_control(self):
        while True:
            print("请输入指令")
            print("1. 直线运动")
            print("2. 旋转运动")
            print("q. 退出")

            choice = input()
            if choice == "1":
                self.line()
            elif choice == "2":
                self.rotate()
            elif choice == "q":
                break

    def line(self):
        while True:
            i = input("输入行进距离，或输入q退出: ")
            if i == "q":
                break
            self.move.line(float(i))

    def rotate(self):
        while True:
            i = input("输入行进距离，或输入q退出: ")
            if i == "q":
                break
            self.move.rotate(float(i))
