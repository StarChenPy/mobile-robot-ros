import asyncio
import time

import rclpy

from ..dao.LiftMotorDao import LiftMotorDao
from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RotateMotorDao import RotateMotorDao
from ..popo.OmsGoal import OmsGoal
from ..popo.Servo import Servo
from ..util import Math
from ..util.Config import Config
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class ArmService:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()

        self.robot_ctrl = RobotCtrlDao(node)
        self.rotary_motor = RotateMotorDao(node)
        self.lift_motor = LiftMotorDao(node)
        self.loop = asyncio.get_event_loop()
        self.task = None

    def back_origin(self, speed=20):
        self.logger.info(f"回原点")

        self.lift_motor.back_origin(speed)
        self.rotary_motor.back_origin(speed)
        self.lift_motor.wait_finish()
        self.lift(1, speed, True)
        self.rotary_motor.wait_finish()

        self.logger.info(f"回原点结束")

    async def __plan(self, goal: OmsGoal, speed=70):
        self.logger.debug(f"开始运动OMS到目标点: {goal}")

        # 电机控制
        if goal.motor_rotary is not None:
            self.rotary_motor.ctrl_motor(goal.motor_rotary, speed)

        if goal.motor_lift is not None:
            self.lift_motor.ctrl_motor(goal.motor_lift, speed)

        # 舵机控制（不需要等待）
        if goal.servo_rotary is not None:
            self.servo_rotary(goal.servo_rotary, publish=False)

        if goal.servo_nod is not None:
            self.servo_nod(goal.servo_nod, publish=False)

        if goal.servo_telescopic is not None:
            self.servo_telescopic(goal.servo_telescopic, publish=False)

        if goal.servo_gripper is not None:
            self.servo_gripper(goal.servo_gripper, publish=False)

        # 统一下发命令
        self.robot_ctrl.publish()

        if goal.sleep:
            await asyncio.sleep(goal.sleep)

    def plan_list(self, goals: list[OmsGoal], speed=70, block=True):
        async def run_plan():
            for goal in goals:
                await self.__plan(goal, speed)
                self.wait_once_finish()
        self.task = self.loop.create_task(run_plan())

        if block:
            self.wait_plan_finish()

    def plan_once(self, goal: OmsGoal, speed=70, block=True):
        self.loop.run_until_complete(self.__plan(goal, speed))
        if block:
            self.wait_once_finish()

    def wait_plan_finish(self):
        if self.task:
            self.loop.run_until_complete(self.task)
            self.task = None
        else:
            self.logger.warn("没有任务在执行")

    def wait_once_finish(self):
        self.lift_motor.wait_finish()
        self.rotary_motor.wait_finish()

    def lift(self, target: float, speed = 80, is_block=True):
        self.lift_motor.ctrl_motor(target, speed)
        if is_block:
            self.lift_motor.wait_finish()

    def rotate(self, target: float, speed = 80, is_block=True):
        target += 1  # 调整偏差
        self.rotary_motor.ctrl_motor(target, speed)
        if is_block:
            self.rotary_motor.wait_finish()

    def wait_finish(self):
        self.lift_motor.wait_finish()
        self.rotary_motor.wait_finish()

    def servo_rotary(self, angle: float, enable=True, publish=True):
        """
        卡爪舵机 旋转
        原 gripper_rz
        """
        if angle < 0:
            angle *= 0.96
        self.__ctrl_servo(Servo.ROTARY, angle, enable, publish)

    def servo_nod(self, angle: float, enable=True, publish=True):
        """
        卡爪舵机 点头(角度)
        原 gripper_ry
        """
        self.__ctrl_servo(Servo.NOD, angle, enable, publish)

    def servo_telescopic(self, distance: float, enable=True, publish=True):
        """
        卡爪舵机 伸缩 ( cm )
        原 telescopic
        """
        self.__ctrl_servo(Servo.TELESCOPIC, distance, enable, publish)

    def servo_gripper(self, distance: float, enable=True, publish=True):
        """
        卡爪舵机 夹合 ( cm )
        原 gripper
        """
        self.__ctrl_servo(Servo.GRIPPER, distance, enable, publish)

    def __ctrl_servo(self, servo: Servo, value: float, enable: bool, publish=True):
        """
        通用舵机控制方法
        @param servo 舵机类型
        @param value 目标值 (角度或距离)
        @param enable 是否使能
        """
        servo_config = Config().get_servo_config()

        duty = 0

        match servo:
            case Servo.NOD:
                config = servo_config["nod"]
                pin = config["pin"]
                min_value = config["min_value"]
                max_value = config["max_value"]
            case Servo.TELESCOPIC:
                config = servo_config["telescopic"]
                pin = config["pin"]
                min_value = config["min_value"]
                max_value = config["max_value"]
            case Servo.GRIPPER:
                config = servo_config["gripper"]
                pin = config["pin"]
                min_value = config["min_value"]
                max_value = config["max_value"]
            case Servo.ROTARY:
                config = servo_config["rotary"]
                pin = config["pin"]
                min_value = config["min_value"]
                max_value = config["max_value"]
            case _:
                self.logger.error(f"未知舵机类型: {servo}")
                return

        type_name = servo.name.lower()
        if not enable:
            self.logger.debug(f'已设置 {type_name} 舵机松使能')
            self.robot_ctrl.write_pwm(pin, 0, publish)
            return

        # 限位处理
        if value < min_value:
            self.logger.warn(f'舵机 {type_name}: {value} 超出最小限位: {min_value}')
            value = min_value
        elif value > max_value:
            self.logger.warn(f'舵机 {type_name}: {value} 超出最大限位: {max_value}')
            value = max_value

        # 转换为PWM duty
        match servo:
            case Servo.NOD:
                config = servo_config["nod"]
                coeff = (config["deg90_duty"] - config["zero_duty"]) / 90.0
                duty = config["zero_duty"] + value * coeff
            case Servo.TELESCOPIC:
                config = servo_config["telescopic"]
                coeff = (config["max_duty"] - config["min_duty"]) / (config["max_value"] - config["min_value"])
                duty = config["max_duty"] - value * coeff
            case Servo.GRIPPER:
                config = servo_config["gripper"]
                coeff = (config["max_duty"] - config["min_duty"]) / config["itinerary"]
                duty = config["min_duty"] + value / 2.0 * coeff
            case Servo.ROTARY:
                config = servo_config["rotary"]
                coeff = (config["deg90_duty"] - config["zero_duty"]) / 90.0
                duty = config["zero_duty"] + value * coeff

        self.robot_ctrl.write_pwm(pin, duty, publish)
