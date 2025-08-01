import rclpy

from .AbstractMotorDao import AbstractMotorDao
from ..popo.MotorCmd import MotorCmd
from ..util.Config import Config
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class RotateMotorDao(AbstractMotorDao):
    def __init__(self, node: rclpy.node.Node):
        super().__init__(node, '/position_motor/rotate_motor/ctrl')

        self.__logger = Logger()

    def ctrl_motor(self, target: float, speed: float):
        """
        电机控制方法，用于旋转电机
        @param target: 目标角度
        @param speed: 速度
        """
        rotate_motor_config = self._get_motor_config()

        # 获取目标值的范围限制
        min_val = rotate_motor_config["min_value"]
        max_val = rotate_motor_config["max_value"]
        if target > max_val:
            self.__logger.warn(f"目标角度 {target} 超过最大值 {max_val}")
            target = max_val
        elif target < min_val:
            self.__logger.warn(f"目标角度 {target} 低于最小值 {min_val}")
            target = min_val

        enc_ppi = 1750.0 * 4
        target_pulses = target * (enc_ppi / 360.0)

        # 调用电机服务
        super()._call_service(MotorCmd.SET_POSITION, target_pulses, speed)
        self.__logger.info(f"已请求旋转电机服务, 目标值 {target}")

    def _get_motor_config(self):
        return Config().get_rotate_motor_config()
