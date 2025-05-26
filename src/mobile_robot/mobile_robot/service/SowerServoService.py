from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RobotDataDao import RobotDataDao
from ..popo.Servo import Servo
from ..util.Config import Config
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class SowerServoService:
    def __init__(self, node):
        self.logger = Logger()
        self.node = node
        self.robot_ctrl = RobotCtrlDao(node)

        self.servo_sower_flag = False

    def disable_all_servo(self):
        self.servo_exit_toggle(0, enable=False)
        self.telescopic_servo(0, enable=False)
        self.servo_sower(enable=False)
        self.servo_knob_rotate(False, enable=False)

    def servo_exit_toggle(self, direction: bool, enable=True):
        """
        播种舵机 出口切换控制
        原 gripper_rz
        """
        speed = 33 if direction else 28
        self.__ctrl_servo(Servo.ROTARY, speed, enable)

    def servo_sower(self, enable=True):
        """
        播种舵机 种子播种控制
        原 gripper_ry
        """
        i = 32 if self.servo_sower_flag else 10
        self.servo_sower_flag = not self.servo_sower_flag
        self.__ctrl_servo(Servo.NOD, i, enable)

    def telescopic_servo(self, distance: float, enable=True):
        """
        播种舵机 伸缩 ( cm )
        原 telescopic
        """
        # 限位处理
        if distance < 0:
            self.logger.warn(f'伸缩 {distance} 超出最小限位: 0')
            distance = 0
        elif distance > 19:
            self.logger.warn(f'伸缩 {distance} 超出最大限位: 19')
            distance = 19

        servo_config = Config().get_servo_config()
        config = servo_config["telescopic"]
        coeff = (config["max_duty"] - config["min_duty"]) / (config["max_value"] - config["min_value"])
        duty = config["max_duty"] - distance * coeff

        self.__ctrl_servo(Servo.TELESCOPIC, duty, enable)

    def servo_knob_rotate(self, switch: bool, enable=True):
        """
        播种舵机 旋钮旋转控制
        原 gripper
        """
        i = 12 if switch else 23.5
        self.__ctrl_servo(Servo.GRIPPER, i, enable)

    def __ctrl_servo(self, servo: Servo, value: float, enable: bool):
        """
        通用舵机控制方法
        @param servo 舵机类型
        @param value 目标值 (角度或距离)
        @param enable 是否使能
        """
        servo_config = Config().get_servo_config()

        pin = 0

        match servo:
            case Servo.NOD:
                config = servo_config["nod"]
                pin = config["pin"]
            case Servo.TELESCOPIC:
                config = servo_config["telescopic"]
                pin = config["pin"]
            case Servo.GRIPPER:
                config = servo_config["gripper"]
                pin = config["pin"]
            case Servo.ROTARY:
                config = servo_config["rotary"]
                pin = config["pin"]

        type_name = servo.name.lower()
        if not enable:
            self.logger.debug(f'已设置 {type_name} 舵机松使能')
            self.robot_ctrl.write_pwm(pin, 0)
            return

        self.robot_ctrl.write_pwm(pin, value)
        self.robot_ctrl.write_pwm(pin, value)
        self.robot_ctrl.write_pwm(pin, value)