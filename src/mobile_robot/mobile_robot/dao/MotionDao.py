import time

import rclpy

import base_motion_ros2.srv

from ..popo.MotionMode import MotionMode
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MotionDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__service = node.create_client(base_motion_ros2.srv.BaseMotion, '/base_motion')

    def __call_service(self, mode: MotionMode, set_point, speed):
        """调用基础运动服务"""
        req = base_motion_ros2.srv.BaseMotion.Request()
        req.motion_mode = mode.value
        req.set_point = float(set_point)

        req.line_param.kp = 1.8
        req.line_param.ti = 0.0
        req.line_param.td = 0.0
        req.line_param.max_vel = float(speed)
        req.line_param.max_acc = 3.0
        req.line_param.low_pass = 0.8
        req.line_param.ek = 0.02
        req.line_param.steady_clk = 5

        match mode:
            case MotionMode.LINE:
                req.rotate_param.kp = 1.8
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = 180.0
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.8
                req.rotate_param.ek = 1.0
                req.rotate_param.steady_clk = 2
            case MotionMode.ROTATE:
                req.rotate_param.kp = 2.8
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = float(speed)
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.7
                req.rotate_param.ek = 1.0
                req.rotate_param.steady_clk = 10

                # 规划器参数 旋转模式
                req.rotate_param.planner_vel = float(speed)   #速度
                req.rotate_param.planner_acc = 600.0          #加速度
                req.rotate_param.planner_decel = 300.0        #减加速度

        return self.__service.call_async(req)

    def line(self, distance: float, speed: float):
        """基础运动: 直线模式"""
        future = self.__call_service(MotionMode.LINE, distance, speed)
        self.__logger.debug(f'已请求直线运动服务 距离 {distance} 速度 {speed}.')

        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10)

        if not future.done():
            self.__logger.warn("请求旋转运动服务超时.")
            return

        if not future.result().success:
            self.__logger.error('无法直线运动!')
            return

    def rotate(self, angle: float, speed: float):
        """基础运动: 旋转模式"""
        future = self.__call_service(MotionMode.ROTATE, angle, speed)
        self.__logger.debug(f'已请求旋转运动服务 角度 {angle} 速度 {speed}.')

        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10)

        if not future.done():
            self.__logger.warn("请求旋转运动服务超时.")
            return

        if not future.result().success:
            self.__logger.error('无法旋转运动!')
            return

    def wait_finish(self):
        """
        等待基础运动完成
        {'feedback': {'motion_mode': 0, 'motion_status': 2, 'error': 0.0}, 'success': True}
        motion_mode: 0:空闲 1:直线 2:旋转
        motion_status: 0:空闲 1:运行中 2:完成
        """
        while rclpy.ok():
            future = self.__call_service(MotionMode.QUERY, 0, 0)

            # 单层等待循环配超时机制
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10)

            if not future.done():
                self.__logger.warn("请求旋转运动服务超时.")
                continue

            feedback = future.result().feedback
            if feedback.motion_mode == 0 and feedback.motion_status == 2:
                self.__logger.debug("运动服务已结束.")
                return

            time.sleep(0.5)

    def stop(self):
        """停止基础运动"""
        self.__logger.debug("停止中...")

        future = self.__call_service(MotionMode.STOP, 0, 0)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10)

        if not future.done():
            self.__logger.warn("请求旋转运动服务超时.")
            return

        if future.result().success:
            self.__logger.debug('停止完成.')
        else:
            self.__logger.error('停止失败.')