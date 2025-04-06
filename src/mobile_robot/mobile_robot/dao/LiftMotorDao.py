import time

import rclpy

import position_motor_ros2.srv
import position_motor_ros2.msg

from ..popo.MotorCmd import MotorCmd
from ..util.ConfigAndParam import ConfigAndParam
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class LiftMotorDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__service = node.create_client(
            position_motor_ros2.srv.CtrlImpl,
            '/position_motor/lift_motor/ctrl')
        self.__service.wait_for_service()

    def __call_service(self, cmd: MotorCmd, target: float, speed: float) -> rclpy.task.Future:
        """
        呼叫电机服务
        @param cmd: 控制命令类型
        @param target: 目标值
        """
        lift_motor_config = ConfigAndParam().get_lift_motor_config()

        request = position_motor_ros2.srv.CtrlImpl.Request(
            cmd=cmd.value,
            target_pose=float(target),
            origin_param=position_motor_ros2.msg.OriginParam(
                back_origin_vel=lift_motor_config["origin_param"]["back_origin_vel"],
                find_origin_cnt=lift_motor_config["origin_param"]["find_origin_cnt"],
                approix_origin_vel=lift_motor_config["origin_param"]["approix_origin_vel"],
                approix_origin_step=lift_motor_config["origin_param"]["approix_origin_step"],
                origin_mode=lift_motor_config["origin_param"]["origin_mode"],
                origin_max_step=lift_motor_config["origin_param"]["origin_max_step"]),
            ctrl_param=position_motor_ros2.msg.AxisParam(
                kp=lift_motor_config["ctrl_param"]["kp"],
                ti=lift_motor_config["ctrl_param"]["ti"],
                td=lift_motor_config["ctrl_param"]["td"],
                max_vel=float(speed),
                max_acc=lift_motor_config["ctrl_param"]["max_acc"],
                low_pass=lift_motor_config["ctrl_param"]["low_pass"],
                ek=lift_motor_config["ctrl_param"]["ek"],
                steady_clk=lift_motor_config["ctrl_param"]["steady_clk"]))

        return self.__service.call_async(request)

    def ctrl_motor(self, target: float, speed: float):
        """
        电机控制方法，用于升降电机
        @param target: 目标高度
        @param speed: 速度
        """
        lift_motor_config = ConfigAndParam().get_lift_motor_config()

        # 获取目标值的范围限制
        min_val = lift_motor_config["min_value"]
        max_val = lift_motor_config["max_value"]
        if target > max_val:
            self.__logger.warn(f"目标高度 {target} 超过最大值 {max_val}")
            target = max_val
        elif target < min_val:
            self.__logger.warn(f"目标高度 {target} 低于最小值 {min_val}")
            target = min_val

        ratio = lift_motor_config["coding_step"] / lift_motor_config["coding_dis"]
        target_pulses = -target * ratio

        # 调用电机服务
        self.__call_service(MotorCmd.SET_POSITION, target_pulses, speed)
        self.__logger.debug(f"已请求移动服务, speed: {speed}")

    def back_origin(self, speed: float):
        self.__call_service(MotorCmd.BACK_ORIGIN, 0, speed)
        self.__logger.debug(f"已请求回原点服务")

    def wait_finish(self):
        while rclpy.ok():
            future = self.__call_service(MotorCmd.READ_FEEDBACK, 0, 0)

            # 单层等待循环配超时机制
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self.__node, timeout_sec=0.2)

            if not rclpy.ok():
                break

            if future.result().feedback.reached:
                self.__logger.debug("电机运动服务已结束")
                return

            time.sleep(0.5)