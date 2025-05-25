import time

from ..popo.ArmMovement import ArmMovement
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService
from ..param import ArmMovement as Movement
from ..service.RobotService import RobotService


class PuchipuchiController:
    def __init__(self, node):
        self.node = node
        self.arm = ArmService(node)
        self.robot = RobotService(node)

        self.arm_movement = ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 0))

    def run(self):
        self.robot.with_robot_connect()
        self.arm.back_origin()

        while True:
            print("请输入指令")
            print("1. 基本控制")
            print("2. 功能演示")
            print("q. 退出")

            choice = input()
            if choice == "1":
                self.basic_control()
            elif choice == "2":
                self.func_control()
            elif choice == "q":
                exit(0)



    def basic_control(self):
        while True:
            print("请输入指令")
            print("1. 夹爪旋转控制")
            print("2. 夹爪抬头控制")
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
                exit(0)

    def servo_rotary(self):
        while True:
            i = input("输入夹爪旋转角度，或输入q退出: ")
            if i == "q":
                break
            self.arm_movement.servo.rotary = int(i)
            self.arm.control(self.arm_movement)

    def servo_nod(self):
        while True:
            i = input("输入夹爪抬头角度，或输入q退出: ")
            if i == "q":
                break
            self.arm_movement.servo.nod = int(i)
            self.arm.control(self.arm_movement)

    def servo_telescopic(self):
        while True:
            i = input("输入夹爪伸缩距离，或输入q退出: ")
            if i == "q":
                break
            self.arm_movement.servo.telescopic = int(i)
            self.arm.control(self.arm_movement)

    def servo_gripper(self):
        while True:
            i = input("输入夹爪张开距离，或输入q退出: ")
            if i == "q":
                break
            self.arm_movement.servo.gripper = int(i)
            self.arm.control(self.arm_movement)

    def arm_lift(self):
        while True:
            i = input("输入升降高度，或输入q退出: ")
            if i == "q":
                break
            self.arm_movement.motor.lift = int(i)
            self.arm.control(self.arm_movement)

    def arm_rotate(self):
        while True:
            i = input("输入旋转角度，或输入q退出: ")
            if i == "q":
                break
            self.arm_movement.motor.rotate = int(i)
            self.arm.control(self.arm_movement)


    def func_control(self):
        while True:
            print("请输入指令")
            print("1. 抓取水果")
            print("2. 抓取果篮")
            print("3. 从车上抓取果篮并放置到地上")
            print("q. 退出")

            choice = input()
            if choice == "1":
                self.grab_fruit()
            if choice == "2":
                self.grab_basket()
            if choice == "3":
                self.grab_basket_to_warehouse()
            elif choice == "q":
                exit(0)

    def grab_fruit(self):
        self.arm.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, -90, 12, 20)))
        self.arm.control(ArmMovement(MotorMovement(180, 32)))
        time.sleep(0.5)
        self.arm.control(ArmMovement(servo=ServoMotor(0, -90, 12, 7)))
        self.arm.control(ArmMovement(MotorMovement(0, 18)))

    def grab_basket(self):
        self.arm.control(ArmMovement(MotorMovement(180, 8), ServoMotor(0, -90, 12, 25)))
        self.arm.control(ArmMovement(MotorMovement(180, 22)))
        self.arm.control(ArmMovement(servo=ServoMotor(0, -90, 12, 19)))
        self.arm.control(ArmMovement(MotorMovement(180, 8), ServoMotor(0, 0, 12, 19)))
        self.arm.control(ArmMovement(MotorMovement(0, 8)))

    def grab_basket_to_warehouse(self):
        for i in range(1, 4):
            Movement.grab_basket_to_warehouse(self.arm, i)
