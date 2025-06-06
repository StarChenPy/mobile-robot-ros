import time

import numpy as np
import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.LiftMotorDao import LiftMotorDao
from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.RotateMotorDao import RotateMotorDao
from ..popo.ArmMovement import ArmMovement
from ..popo.Direction import Direction
from ..popo.MotorMovement import MotorMovement
from ..popo.Servo import Servo
from ..popo.ServoMotor import ServoMotor
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

    def grab_fruit(self, height: float, direction: Direction.LEFT or Direction.RIGHT):
        """抓墙上水果"""
        self.__logger.info(f"收到的升降距离为 {height}")
        if height > 29:
            telescopic = 8
            nod = -60
            height -= 10
        else:
            telescopic = 1.5
            nod = 0

        # 计算伸缩要伸出的距离
        default_distance_from_wall = 0.33
        distance_from_wall = 0

        if direction == Direction.LEFT:
            distance_from_wall = self.__robot_data.get_ir_left()
        elif direction == Direction.RIGHT:
            distance_from_wall = self.__robot_data.get_ir_right()

        if distance_from_wall and 0.5 > distance_from_wall > 0.1:
            dis = (distance_from_wall - default_distance_from_wall) * 100
            telescopic += dis
            self.__logger.info(f"伸缩距离计算为 {telescopic}")
        else:
            self.__logger.warn(f"红外数据不可信，尝试使用雷达数据")
            distance_from_wall = self.__radar.get_distance_from_wall(direction)
            if distance_from_wall and 0.4 > distance_from_wall > 0.1:
                dis = (distance_from_wall - default_distance_from_wall) * 100
                telescopic += dis
                self.__logger.info(f"伸缩距离计算为 {telescopic}")
            else:
                self.__logger.warn(f"雷达数据不可信，使用默认值")

        # 计算要旋转的角度
        angle = self.__radar.get_angle_from_wall(direction)

        if angle == 0 or angle > 10:
            self.__logger.warn("雷达角度不可用，使用Odom Yaw进行机械臂角度矫正")
            odom_yaw = self.__robot_data.get_robot_data().odom.y
            angle = 90 - (odom_yaw % 90)

        if direction == Direction.LEFT:
            arm_pos = 90 - angle
        elif direction == Direction.RIGHT:
            arm_pos = -90 - angle
        else:
            raise ValueError("不可用的Direction")

        self.__logger.info(f"旋转角度计算为 {arm_pos}")

        # 准备抓
        self.control(ArmMovement(MotorMovement(arm_pos, 18), ServoMotor(0, nod, telescopic, 22)))
        self.control(ArmMovement(MotorMovement(arm_pos, height), ServoMotor(0, nod, telescopic, 22)))
        # 夹合
        self.control(ArmMovement(MotorMovement(arm_pos, height), ServoMotor(0, nod, telescopic, 6.5)))
        time.sleep(0.5)
        # 提起
        self.control(ArmMovement(MotorMovement(arm_pos, 18), ServoMotor(0, nod, telescopic, 6.5)))

    def control(self, movement: ArmMovement, speed=60.0, is_block=True):
        self.__logger.debug(f"机械臂控制 {movement}")

        if movement.motor is not None:
            self.lift(movement.motor.lift, speed, False)
            self.rotate(movement.motor.rotate, speed, False)

        if movement.servo is not None:
            self.nod_servo(movement.servo.nod)
            self.telescopic_servo(movement.servo.telescopic)
            self.gripper_servo(movement.servo.gripper)
            self.rotary_servo(movement.servo.rotary)

        if is_block and movement.motor is not None:
            self.__lift_motor.wait_finish()
            self.__rotate_motor.wait_finish()

    def back_origin(self, speed=20):
        self.__logger.info(f"回原点")

        self.__lift_motor.back_origin(speed)
        self.__rotate_motor.back_origin(speed)
        self.__lift_motor.wait_finish()
        self.__lift_motor.ctrl_motor(0.5, speed)
        self.__lift_motor.wait_finish()
        self.__rotate_motor.wait_finish()

        self.__logger.info(f"回原点结束")

    def lift(self, target: float, speed: float, is_block):
        self.__lift_motor.ctrl_motor(target, speed)
        if is_block:
            self.__lift_motor.wait_finish()

    def rotate(self, target: float, speed: float, is_block):
        self.__rotate_motor.ctrl_motor(target, speed)
        if is_block:
            self.__lift_motor.wait_finish()

    def disable_servo(self):
        self.nod_servo(0, enable=False)
        self.telescopic_servo(0, enable=False)
        self.gripper_servo(0, enable=False)
        self.rotary_servo(0, enable=False)

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
        self.__ctrl_servo(Servo.TELESCOPIC, distance, enable, 3)

    def gripper_servo(self, distance: float, enable=True):
        """
        卡爪舵机 夹合 ( cm )
        原 gripper
        """
        self.__ctrl_servo(Servo.GRIPPER, distance, enable, 1)

    def __ctrl_servo(self, servo: Servo, value: float, enable: bool, decelerate=2):
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
                duty_array = np.linspace(prev_duty, duty, int(decelerate * 30))
                for d in duty_array:
                    self.__robot_ctrl.write_pwm(pin, d.item())
                    time.sleep(difference / (decelerate * 1000))
            else:
                self.__robot_ctrl.write_pwm(pin, duty)
        else:
            self.__logger.debug(f'舵机 {type_name} 已经在目标位置，无需调整')
