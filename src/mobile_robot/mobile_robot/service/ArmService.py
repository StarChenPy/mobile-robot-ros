import threading
import time

import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.LiftMotorDao import LiftMotorDao
from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.RotateMotorDao import RotateMotorDao
from ..popo.Servo import Servo
from ..util import Math
from ..util.Config import Config
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class ArmService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__lift_motor = LiftMotorDao(node)
        self.__rotate_motor = RotateMotorDao(node)
        self.__robot_ctrl = RobotCtrlDao(node)
        self.__robot_data = RobotDataDao(node)
        self.__radar = LaserRadarDao(node)

    def back_origin(self, speed=20):
        self.__logger.info(f"回原点")

        self.__lift_motor.back_origin(speed)
        self.__rotate_motor.back_origin(speed)
        self.__lift_motor.wait_finish()
        self.lift(1, speed, True)
        self.__rotate_motor.wait_finish()

        self.__logger.info(f"回原点结束")

    def lift(self, target: float, speed = 50, is_block=True):
        self.__lift_motor.ctrl_motor(target, speed)
        if is_block:
            self.__lift_motor.wait_finish()

    def rotate(self, target: float, speed = 60, is_block=True):
        target += 2  # 调整偏差
        self.__rotate_motor.ctrl_motor(target, speed)
        if is_block:
            self.__rotate_motor.wait_finish()

    def wait_finish(self):
        self.__lift_motor.wait_finish()
        self.__rotate_motor.wait_finish()

    def rotary_servo(self, angle: float, enable=True):
        """
        卡爪舵机 旋转
        原 gripper_rz
        """
        self.__ctrl_servo(Servo.ROTARY, angle, enable)

    def nod_servo(self, angle: float, enable=True):
        """
        卡爪舵机 点头(角度)
        原 gripper_ry
        """
        self.__ctrl_servo(Servo.NOD, angle, enable)

    def telescopic_servo(self, distance: float, enable=True):
        """
        卡爪舵机 伸缩 ( cm )
        原 telescopic
        """
        self.__ctrl_servo(Servo.TELESCOPIC, distance, enable, 1)

    def gripper_servo(self, distance: float, enable=True):
        """
        卡爪舵机 夹合 ( cm )
        原 gripper
        """
        self.__ctrl_servo(Servo.GRIPPER, distance, enable)

    def __ctrl_servo(self, servo: Servo, value: float, enable: bool, decelerate=0):
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
                self.__logger.error(f"未知舵机类型: {servo}")
                return

        type_name = servo.name.lower()
        if not enable:
            self.__logger.debug(f'已设置 {type_name} 舵机松使能')
            self.__robot_ctrl.write_pwm(pin, 0)
            return

        # 限位处理
        if value < min_value:
            self.__logger.warn(f'舵机 {type_name}: {value} 超出最小限位: {min_value}')
            value = min_value
        elif value > max_value:
            self.__logger.warn(f'舵机 {type_name}: {value} 超出最大限位: {max_value}')
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

        prev_duty = self.__robot_ctrl.read_pwm(pin)
        if prev_duty != duty:
            self.__logger.info(f'设置 {type_name} 舵机: {value} (duty: {duty})')

            if decelerate > 0 and prev_duty != 0:
                difference = abs(prev_duty - duty)
                steps = int(decelerate * 30)
                duty_array = Math.ease_in_out_interp(prev_duty, duty, steps)
                for d in duty_array:
                    self.__robot_ctrl.write_pwm(pin, d.item())
                    time.sleep(difference / (decelerate * 1000))
            else:
                self.__robot_ctrl.write_pwm(pin, duty)
                time.sleep(0.5)
        else:
            self.__logger.debug(f'舵机 {type_name} 已经在目标位置，无需调整')
