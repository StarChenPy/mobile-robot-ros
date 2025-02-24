import rclpy

from ..dao.LiftMotorDao import LiftMotorDao
from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RotateMotorDao import RotateMotorDao
from ..popo.Servo import Servo
from ..robot.param.arm_movement import ArmMovementParam
from ..util.Config import Config
from ..util.Singleton import singleton


@singleton
class ArmService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__lift_motor = LiftMotorDao(node)
        self.__rotate_motor = RotateMotorDao(node)
        self.__robot_ctrl = RobotCtrlDao(node)

    def arm_control(self, movement: ArmMovementParam, is_block=False):
        self.__logger.info(f"[机械臂] 机械臂控制 {movement.name}")

        if movement.value.motor is not None:
            self.lift(movement.value.motor.lift, is_block)
            self.rotate(movement.value.motor.rotate, is_block)
            self.__lift_motor.wait_finish()
            self.__rotate_motor.wait_finish()

        if movement.value.servo is not None:
            self.nod_servo(movement.value.servo.nod)
            self.telescopic_servo(movement.value.servo.telescopic)
            self.gripper_servo(movement.value.servo.gripper)
            self.rotary_servo(movement.value.servo.rotary)

    def back_origin(self, speed=20):
        self.__logger.info(f"[机械臂] 回原点")

        self.__lift_motor.back_origin(speed)
        self.__rotate_motor.back_origin(speed)
        self.__lift_motor.wait_finish()
        self.__lift_motor.ctrl_motor(0.5, 20)
        self.__lift_motor.wait_finish()
        self.__rotate_motor.wait_finish()

    def lift(self, target: float, speed: float = 20, is_block=True):
        self.__lift_motor.ctrl_motor(target, speed)
        if is_block:
            self.__lift_motor.wait_finish()

    def rotate(self, target: float, speed: float = 20, is_block=True):
        self.__rotate_motor.ctrl_motor(target, speed)
        if is_block:
            self.__lift_motor.wait_finish()

    def rotary_servo(self, angle, enable=True):
        """
        卡爪舵机 旋转
        原 gripper_rz
        """
        self.__ctrl_servo(Servo.ROTARY, angle, enable)

    def nod_servo(self, angle=0, enable=True):
        """
        卡爪舵机 点头(角度)
        原 gripper_ry
        """
        self.__ctrl_servo(Servo.NOD, angle, enable)

    def telescopic_servo(self, distance=0, enable=True):
        """
        卡爪舵机 伸缩 ( cm )
        原 telescopic
        """
        self.__ctrl_servo(Servo.TELESCOPIC, distance, enable)

    def gripper_servo(self, distance=0, enable=True):
        """
        卡爪舵机 夹合 ( cm )
        原 gripper
        """
        self.__ctrl_servo(Servo.GRIPPER, distance, enable)

    def __ctrl_servo(self, servo: Servo, value: float, enable: bool):
        """
        通用舵机控制方法
        @param servo 舵机类型
        @param value 目标值 (角度或距离)
        @param enable 是否使能
        """
        servo_config = Config().get_servo_config()

        pin = 0
        min_value = 0
        max_value = 0
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

        type_name = servo.name.lower()
        if not enable:
            self.__logger.debug(f'[舵机] 已设置 {type_name} 舵机松使能')
            self.__robot_ctrl.write_pwm(pin, 0)
            return

        # 限位处理
        if value < min_value:
            self.__logger.warn(f'[舵机] 目标 {type_name}: {value} 超出最小限位: {min_value}')
            value = min_value
        elif value > max_value:
            self.__logger.warn(f'[舵机] 目标 {type_name}: {value} 超出最大限位: {max_value}')
            value = max_value

        # 转换为PWM duty
        match servo:
            case Servo.NOD:
                config = servo_config["gripper"]
                coeff = (config["deg90_duty"] - config["zero_duty"]) / 90.0
                duty = config["zero_duty"] + value * coeff
            case Servo.TELESCOPIC:
                config = servo_config["telescopic"]
                coeff = (config["max_duty"] - config["min_duty"]) / config["itinerary"]
                duty = config["max_duty"] - value * coeff
            case Servo.GRIPPER:
                config = servo_config["gripper"]
                coeff = (config["max_duty"] - config["min_duty"]) / config["itinerary"]
                duty = config["min_duty"] + value / 2.0 * coeff
            case Servo.ROTARY:
                config = servo_config["gripper"]
                coeff = (config["deg90_duty"] - config["zero_duty"]) / 90.0
                duty = config["zero_duty"] + value * coeff

        self.__robot_ctrl.write_pwm(pin, duty)