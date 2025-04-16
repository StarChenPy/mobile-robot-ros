import rclpy

from .AbstractMotorDao import AbstractMotorDao
from ..popo.MotorCmd import MotorCmd
from ..util.ConfigAndParam import ConfigAndParam
from ..util.Singleton import singleton


@singleton
class RotateMotorDao(AbstractMotorDao):
    def __init__(self, node: rclpy.node.Node):
        super().__init__(node, '/position_motor/rotate_motor/ctrl')

    def ctrl_motor(self, target: float, speed: float):
        """
        电机控制方法，用于旋转电机
        @param target: 目标角度
        @param speed: 速度
        """
        rotate_motor_config = self.get_motor_config()

        # 获取目标值的范围限制
        min_val = rotate_motor_config["min_value"]
        max_val = rotate_motor_config["max_value"]
        if target > max_val:
            self.__logger.warn(f"目标角度 {target} 超过最大值 {max_val}")
            target = max_val
        elif target < min_val:
            self.__logger.warn(f"目标角度 {target} 低于最小值 {min_val}")
            target = min_val

        enc_ppi = 1750.0 * 3.9
        target_pulses = target * (enc_ppi / 360.0)

        # 调用电机服务
        self.__call_service(MotorCmd.SET_POSITION, target_pulses, speed)
        self.__logger.debug(f"已请求服务")

    def get_motor_config(self):
        return ConfigAndParam().get_rotate_motor_config()
