import time

import rclpy
from rclpy.client import Client
from rclpy.node import Node

from position_motor_ros2 import srv
from . import io_impl
from ..param.arm_param import *


class ArmImpl:
    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()
        self.__io = io_impl.IoImpl.instance(node)

        self.__srv_rotate_motor = node.create_client(srv.CtrlImpl, '/position_motor/rotate_motor/ctrl')
        self.__srv_lift_motor = node.create_client(srv.CtrlImpl, '/position_motor/lift_motor/ctrl')

        self.__logger.info("[机械臂] 初始化完成")

    # ===============================电机部分===============================

    def __call_motor(self, srv_client: Client, cmd: MotorCmd, target: float, origin: msg.OriginParam,
                     ctrl: msg.AxisParam):
        """
        通过服务控制电机运动
        @param srv_client: 具体的电机服务
        @param cmd: 控制命令类型
        @param target: 目标值
        @param origin: 回原点参数
        @param ctrl: 轴闭环控制参数、滤波器参数与跳出条件
        """
        request = srv.CtrlImpl.Request(
            cmd=cmd.value,
            target_pose=float(target),
            origin_param=origin,
            ctrl_param=ctrl
        )
        return srv_client.call_async(request)

    def _ctrl_motor(self, motor_type: Motor, cmd: MotorCmd, target: float, speed: float, is_block: bool):
        """
        通用的电机控制方法，用于旋转和升降电机
        @param motor_type: 电机类型（旋转或升降）
        @param cmd: 控制命令类型
        @param target: 目标值（角度或高度）
        @param speed: 速度
        @param is_block: 是否阻塞等待运动完成
        """
        motor_param = motor_type.value
        self.__logger.debug(f"[机械臂电机] {motor_type.name} 电机 命令: {cmd} 目标: {target} 速度: {speed}")

        # 设置最大速度
        motor_param.ctrl_param.max_vel = float(speed)

        # 获取目标值的范围限制
        min_val, max_val = motor_param.min_value, motor_param.max_value
        if target > max_val:
            self.__logger.warning(f"[机械臂电机] 目标 {motor_type.name} 角度/高度 {target} 超过最大值 {max_val}")
            target = max_val
        elif target < min_val:
            self.__logger.warning(f"[机械臂电机] 目标 {motor_type.name} 角度/高度 {target} 低于最小值 {min_val}")
            target = min_val

        # 根据电机类型转换目标值
        if motor_type == Motor.ROTATE:
            enc_ppi = 1750.0 * 4.0
            target_pulses = target * (enc_ppi / 360.0)
        elif motor_type == Motor.LIFT:
            ratio = motor_param.coding_step / motor_param.coding_dis
            target_pulses = -target * ratio
        else:
            self.__logger.error(f"未知的电机类型: {motor_type}")
            return None

        # 调用电机服务
        srv_client = self.__srv_rotate_motor if motor_type == Motor.ROTATE else self.__srv_lift_motor
        future = self.__call_motor(
            srv_client, cmd, target_pulses, motor_param.origin_param, motor_param.ctrl_param
        )
        self.__logger.info(f"[机械臂电机] {motor_type.name} 电机请求已发送")

        # 阻塞等待运动完成
        if is_block:
            self.wait_motor_finish(motor_type)

        return future

    def wait_motor_finish(self, motor_type: Motor):
        client = self.__srv_lift_motor if motor_type == Motor.LIFT else self.__srv_rotate_motor
        flag = True

        while flag:
            future = self.__call_motor(client, MotorCmd.READ_FEEDBACK, 0, motor_type.value.origin_param,
                                       motor_type.value.ctrl_param)

            # 这里鉴于非同步式的特殊性质，需要双层while循环请求并验证是否有效
            while rclpy.ok():
                rclpy.spin_once(self.__node)

                if not future.done():
                    continue

                if future.result().feedback.reached:
                    self.__logger.info(f"[机械臂电机] {motor_type.name} 电机运动已完成")
                    flag = False
                    break
                else:
                    break
            time.sleep(0.2)

    def ctrl_rotate_motor(self, cmd: MotorCmd, angle=0, speed=50.0, is_block=True):
        """
        控制旋转电机运动
        @param cmd: 控制命令类型
        @param angle: 旋转角度
        @param speed: 旋转速度
        @param is_block: 是否阻塞
        """
        return self._ctrl_motor(Motor.ROTATE, cmd, angle, speed, is_block)

    def ctrl_lift_motor(self, cmd: MotorCmd, height=0, speed=30.0, is_block=True):
        """
        控制升降电机运动
        @param cmd: 控制命令类型
        @param height: 升降距离
        @param speed: 升降速度
        @param is_block: 是否阻塞
        """
        return self._ctrl_motor(Motor.LIFT, cmd, height, speed, is_block)

    # ===============================舵机部分===============================

    def set_servo(self, servo_param_type: Servo, value, enable):
        """
        通用舵机控制方法
        @param servo_param_type 舵机参数类型
        @param value 目标值 (角度或距离)
        @param enable 是否使能
        """

        servo_param = servo_param_type.value
        type_name = servo_param_type.name.lower()

        if not enable:
            self.__logger.info(f'[机械臂舵机] 已设置 {type_name} 舵机松使能')
            self.__io.write_pwm(servo_param.pin, 0)
            return

        self.__logger.debug(f'[机械臂舵机] 设置 {type_name}: {value}')

        # 限位处理
        if value < servo_param.min_value:
            self.__logger.warn(f'[机械臂舵机] 目标 {type_name}: {value} 超出最小限位: {servo_param.min_value}')
            value = servo_param.min_value
        elif value > servo_param.max_value:
            self.__logger.warn(f'[机械臂舵机] 目标 {type_name}: {value} 超出最大限位: {servo_param.max_value}')
            value = servo_param.max_value

        # 转换为PWM duty
        match servo_param_type:
            case Servo.TELESCOPIC_SERVO:
                coeff = (servo_param.max_duty - servo_param.min_duty) / servo_param.itinerary
                duty = servo_param.max_duty - value * coeff
            case Servo.GRIPPER_SERVO:
                coeff = (servo_param.max_duty - servo_param.min_duty) / servo_param.itinerary
                duty = servo_param.min_duty + value / 2.0 * coeff
            case _:
                coeff = (servo_param.deg90_duty - servo_param.zero_duty) / 90.0
                duty = servo_param.zero_duty + value * coeff

        self.__io.write_pwm(servo_param.pin, duty)

    def rotary_servo(self, angle, enable=True):
        """
        卡爪舵机 旋转
        原 gripper_rz
        """
        self.set_servo(Servo.ROTARY_SERVO, angle, enable)

    def nod_servo(self, angle=0, enable=True):
        """
        卡爪舵机 点头(角度)
        原 gripper_ry
        """
        self.set_servo(Servo.NOD_SERVO, angle, enable)

    def telescopic_servo(self, distance=0, enable=True):
        """
        卡爪舵机 伸缩 ( cm )
        原 telescopic
        """
        self.set_servo(Servo.TELESCOPIC_SERVO, distance, enable)

    def gripper_servo(self, distance=0, enable=True):
        """
        卡爪舵机 夹合 ( cm )
        原 gripper
        """
        self.set_servo(Servo.GRIPPER_SERVO, distance, enable)
