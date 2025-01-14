from rclpy.client import Client
from rclpy.node import Node

from position_motor_ros2 import srv
from . import io_impl
from ..param.arm_param import *


class ArmImpl:
    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()
        self.__io = io_impl.IoImpl(node)

        self.__srv_rotate_motor = node.create_client(srv.CtrlImpl, '/position_motor/rotate_motor/ctrl')
        self.__srv_lift_motor = node.create_client(srv.CtrlImpl, '/position_motor/lift_motor/ctrl')

        self.__logger.info("[机械臂] 初始化完成")

    # ===============================电机部分===============================

    def __call_motor(self, srv_client: Client, cmd: RotateMotorCmd, target: float, origin: msg.OriginParam,
                     ctrl: msg.AxisParam):
        """
        通过服务控制电机运动
        @param srv_client 具体的电机服务
        @param cmd 控制命令类型
        @param target 目标值
        @param origin 回原点参数
        @param ctrl 轴闭环控制参数、滤波器参数与跳出条件
        """

        request = srv.CtrlImpl.Request()
        request.cmd = cmd.value
        request.target_pose = target
        request.origin_param = origin
        request.ctrl_param = ctrl

        self.__logger.debug(f"[机械臂电机] 正在调用 {srv_client.srv_name}")
        future = srv_client.call_async(request)
        self.__logger.debug(f"[机械臂电机] 调用完成 {srv_client.srv_name}")

        return future

    def ctrl_rotate_motor(self, cmd: RotateMotorCmd, angle=0, speed=50.0, block=True):
        """
        控制旋转电机运动
        @param cmd 控制命令类型
        @param angle 旋转角度
        @param speed 旋转速度
        @param block 是否阻塞
        """

        self.__logger.info(f'[机械臂电机] 旋转电机 命令: {cmd} 目标角度: {angle} 速度: {speed}')

        motor_param = Motor.ROTATE.value
        motor_param.ctrl_param.max_vel = float(speed)

        if angle > motor_param.max_value:
            self.__logger.warn(f"[机械臂电机] 目标旋转电机角度 {angle} 超过最大值 {motor_param.max_value}")
            angle = motor_param.max_value
        elif angle < motor_param.min_value:
            self.__logger.warn(f"[机械臂电机] 目标旋转电机角度 {angle} 超过最小值 {motor_param.min_value}")
            angle = motor_param.min_value

        enc_ppi = 1750.0 * 4.0
        angle = angle * (enc_ppi / 360.0)

        future = self.__call_motor(self.__srv_rotate_motor, cmd, angle, motor_param.origin_param,
                                   motor_param.ctrl_param)
        self.__logger.warn(f"[机械臂电机] 旋转电机请求已发送")
        if not block:
            while not future.done():
                pass
        return future

    def ctrl_lift_motor(self, cmd: RotateMotorCmd, height=0, speed=50.0, block=True):
        """
        控制升降电机运动
        @param cmd 控制命令类型
        @param height 升降距离
        @param speed 升降速度
        @param block 是否阻塞
        """

        self.__logger.info(f'[机械臂电机] 升降电机 命令: {cmd} 目标高度: {height} 速度: {speed}')

        motor_param = Motor.LIFT.value
        motor_param.ctrl_param.max_vel = float(speed)

        if height > motor_param.max_value:
            self.__logger.warn(f"[机械臂电机] 目标升降电机距离 {height} 超过最大值 {motor_param.max_value}")
            height = motor_param.max_value
        elif height < motor_param.min_value:
            self.__logger.warn(f"[机械臂电机] 目标升降电机距离 {height} 超过最大值 {motor_param.min_value}")
            height = motor_param.min_value

        ratio = motor_param.coding_step / motor_param.coding_dis  #得到每个cm多少个脉冲
        height = -height * ratio  #计算目标脉冲

        future = self.__call_motor(self.__srv_lift_motor, cmd, height, motor_param.origin_param, motor_param.ctrl_param)
        self.__logger.warn(f"[机械臂电机] 升降电机请求已发送")
        if not block:
            while not future.done():
                pass
        return future

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

        self.__logger.info(f'[机械臂舵机] 设置 {type_name}: {value}')

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
