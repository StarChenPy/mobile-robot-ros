import time
from abc import ABC, abstractmethod

import rclpy

import position_motor_ros2.msg
import position_motor_ros2.srv
from ..popo.MotorCmd import MotorCmd
from ..util.Logger import Logger


class AbstractMotorDao(ABC):
    def __init__(self, node: rclpy.node.Node, src_name: str):
        self.__node = node
        self.__logger = Logger()

        self.__service = node.create_client(position_motor_ros2.srv.CtrlImpl, src_name)

    def _call_service(self, cmd: MotorCmd, target: float, speed: float) -> rclpy.task.Future:
        """
        呼叫电机服务
        @param cmd: 控制命令类型
        @param target: 目标值
        """
        self.__service.wait_for_service()

        motor_config = self._get_motor_config()

        request = position_motor_ros2.srv.CtrlImpl.Request(
            cmd=cmd.value,
            target_pose=float(target),
            origin_param=position_motor_ros2.msg.OriginParam(
                back_origin_vel=motor_config["origin_param"]["back_origin_vel"],
                find_origin_cnt=motor_config["origin_param"]["find_origin_cnt"],
                approix_origin_vel=motor_config["origin_param"]["approix_origin_vel"],
                approix_origin_step=motor_config["origin_param"]["approix_origin_step"],
                origin_mode=motor_config["origin_param"]["origin_mode"],
                origin_max_step=motor_config["origin_param"]["origin_max_step"]),
            ctrl_param=position_motor_ros2.msg.AxisParam(
                kp=motor_config["ctrl_param"]["kp"],
                ti=motor_config["ctrl_param"]["ti"],
                td=motor_config["ctrl_param"]["td"],
                max_vel=float(speed),
                max_acc=motor_config["ctrl_param"]["max_acc"],
                low_pass=motor_config["ctrl_param"]["low_pass"],
                ek=motor_config["ctrl_param"]["ek"],
                steady_clk=motor_config["ctrl_param"]["steady_clk"]))

        return self.__service.call_async(request)

    @abstractmethod
    def ctrl_motor(self, target: float, speed: float):
        pass

    @abstractmethod
    def _get_motor_config(self):
        pass

    def back_origin(self, speed: float):
        self._call_service(MotorCmd.BACK_ORIGIN, 0, speed)
        self.__logger.debug(f"已请求回原点服务")

    def wait_finish(self):
        while rclpy.ok():
            future = self._call_service(MotorCmd.READ_FEEDBACK, 0, 0)

            # 单层等待循环配超时机制
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10)

            if not future.done():
                self.__logger.warn("请求电机运动服务超时.")
                continue

            if future.result().feedback.reached:
                self.__logger.debug("电机运动服务已结束")
                return
