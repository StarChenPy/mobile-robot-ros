import time

import rclpy

import position_motor_ros2.srv
import position_motor_ros2.msg

from ..popo.MotorCmd import MotorCmd
from ..util.Config import Config
from ..util.Singleton import singleton


@singleton
class RotateMotorDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()
        self.__service = node.create_client(
            position_motor_ros2.srv.CtrlImpl,
            '/position_motor/rotate_motor/ctrl')

        self.__service.wait_for_service()

    def __call_service(self, cmd: MotorCmd, target: float, speed: float) -> rclpy.task.Future:
        """
        呼叫电机服务
        @param cmd: 控制命令类型
        @param target: 目标值
        """
        rotate_motor_config = Config().get_rotate_motor_config()

        request = position_motor_ros2.srv.CtrlImpl.Request(
            cmd=cmd.value,
            target_pose=float(target),
            origin_param=position_motor_ros2.msg.OriginParam(
                back_origin_vel=rotate_motor_config["origin_param"]["back_origin_vel"],
                find_origin_cnt=rotate_motor_config["origin_param"]["find_origin_cnt"],
                approix_origin_vel=rotate_motor_config["origin_param"]["approix_origin_vel"],
                approix_origin_step=rotate_motor_config["origin_param"]["approix_origin_step"],
                origin_mode=rotate_motor_config["origin_param"]["origin_mode"],
                origin_max_step=rotate_motor_config["origin_param"]["origin_max_step"]),
            ctrl_param=position_motor_ros2.msg.AxisParam(
                kp=rotate_motor_config["ctrl_param"]["kp"],
                ti=rotate_motor_config["ctrl_param"]["ti"],
                td=rotate_motor_config["ctrl_param"]["td"],
                max_vel=float(speed),
                max_acc=rotate_motor_config["ctrl_param"]["max_acc"],
                low_pass=rotate_motor_config["ctrl_param"]["low_pass"],
                ek=rotate_motor_config["ctrl_param"]["ek"],
                steady_clk=rotate_motor_config["ctrl_param"]["steady_clk"]))

        return self.__service.call_async(request)

    def ctrl_motor(self, target: float, speed: float):
        """
        电机控制方法，用于旋转电机
        @param target: 目标角度
        @param speed: 速度
        """
        rotate_motor_config = Config().get_rotate_motor_config()

        # 获取目标值的范围限制
        min_val = rotate_motor_config["min_value"]
        max_val = rotate_motor_config["max_value"]
        if target > max_val:
            self.__logger.warning(f"[旋转电机] 目标角度 {target} 超过最大值 {max_val}")
            target = max_val
        elif target < min_val:
            self.__logger.warning(f"[旋转电机] 目标角度 {target} 低于最小值 {min_val}")
            target = min_val

        enc_ppi = 1750.0 * 3.9
        target_pulses = target * (enc_ppi / 360.0)

        # 调用电机服务
        self.__call_service(MotorCmd.SET_POSITION, target_pulses, speed)
        self.__logger.debug(f"[旋转电机] 已请求服务")

    def back_origin(self, speed: float):
        self.__call_service(MotorCmd.BACK_ORIGIN, 0, speed)
        self.__logger.debug(f"[旋转电机] 已请求回原点服务")

    def wait_finish(self):
        while rclpy.ok():
            future = self.__call_service(MotorCmd.READ_FEEDBACK, 0, 0)

            # 单层等待循环配超时机制
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self.__node, timeout_sec=0.2)

            if not rclpy.ok():
                break

            if future.result().feedback.reached:
                self.__logger.debug("[旋转电机] 电机运动服务已结束")
                return

            time.sleep(0.5)