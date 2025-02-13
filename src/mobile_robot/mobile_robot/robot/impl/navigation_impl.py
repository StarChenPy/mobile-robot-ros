import math
import threading

import rclpy

from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.action import ActionClient
from .io_impl import IoImpl
from base_motion_ros2.srv import BaseMotion
from base_nav2.action import NavCMD
from chassis_msgs.srv import ResetOdom
from ..data_type import *


class NavigationImpl:
    __goal_handle = None
    __is_navigating = False

    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()
        self.__io = IoImpl.instance(node)

        self.__motion_srv = node.create_client(BaseMotion, '/base_motion')
        self.__odom_srv = node.create_client(ResetOdom, '/chassis/reset_odom')
        self.__navigation_action = ActionClient(node, NavCMD, '/nav2_action')

        self.__logger.info("[导航接口] 初始化完成.")

        self.__mtx_navigation = threading.Lock()  #base_nav2 互斥锁

    # ===============================导航部分===============================

    def init_pose(self, pose: Pose, mode=ResetOdomMode.RESET_ALL) -> None:
        """初始化机器人位置，支持重置odom不同模式"""
        self.__logger.info(f"[导航接口] 初始化机器人位置 [{pose.x}, {pose.y}, {pose.yaw}] 模式为 {mode.name}")

        req = ResetOdom.Request()
        req.clear_mode = mode.value
        req.x = float(pose.x)
        req.y = float(pose.y)
        radian = math.radians(pose.yaw)
        req.theta = float(radian)

        future = self.__odom_srv.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            result = future.result()
            if result.success:
                self.__logger.info("[导航接口] 重置 Odometry 成功")
                break
            else:
                self.__logger.error("[导航接口] 重置 Odometry 错误")
                break

    def navigation(self, points: list[Pose], linear_speed=0.5, rotation_speed=2.5, is_block=True) -> None:
        """
        路径跟随: 输入路径点、最终角度等参数，发送导航请求
        @param points 路径坐标点[x,y]
        @param linear_speed 最大线速度m/s
        @param rotation_speed 最大旋转速度m/s
        @param is_block 是否阻塞
        """

        goal_msg = NavCMD.Goal()

        for p in points:
            pose2d = Pose2D()
            pose2d.x = float(p.x)
            pose2d.y = float(p.y)
            pose2d.theta = 0.0
            goal_msg.points.append(pose2d)

        # 这里要获取导航最后一个点的角度并赋给heading
        goal_msg.heading = float(points[-1].yaw)
        goal_msg.back = False
        goal_msg.linear_vel = float(linear_speed)
        goal_msg.rotation_vel = float(rotation_speed)

        self.__logger.info("[导航接口] 等待导航服务")
        self.__navigation_action.wait_for_server()
        self.__logger.info("[导航接口] 正在发送新的导航请求")

        goal_handle = self.__navigation_action.send_goal_async(goal_msg)
        goal_handle.add_done_callback(self.__goal_response_callback)

        with self.__mtx_navigation:
            self.__is_navigating = True

        if is_block:
            self.wait_navigation_finish()

    def __goal_response_callback(self, future) -> None:
        """处理Goal响应"""
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.__logger.error("[导航接口] 服务端拒绝本次Goal请求!")
            return

        self.__logger.info("[导航接口] 正在执行导航...")
        __get_result_future = self.__goal_handle.get_result_async()
        __get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, _) -> None:
        """处理Goal完成回调"""
        self.__logger.info(f"[导航接口] 导航完成")

        with self.__mtx_navigation:
            self.__is_navigating = False

    def wait_navigation_finish(self) -> None:
        """等待导航完成"""
        self.__logger.info("[导航接口] 等待导航结束中...")
        while rclpy.ok():
            rclpy.spin_once(self.__node)
            with self.__mtx_navigation:
                if not self.__is_navigating:
                    break
        self.__logger.info("[导航接口] 等待导航结束")

    def cancel_navigation(self) -> None:
        """取消路径跟随"""
        while self.__goal_handle is None:
            rclpy.spin_once(self.__node)

        assert self.__goal_handle is not None
        self.__logger.info("[导航接口] 取消导航...")
        self.__goal_handle.cancel_goal_async()

    def get_navigation_state(self) -> bool:
        """
        获取导航状态
        @return True 正在导航
        """
        self.__mtx_navigation.acquire()
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        state = self.__is_navigating
        self.__mtx_navigation.release()
        return state

    # ===============================基础运动部分===============================

    def __call_srv_base_motion(self, mode: BaseMotionMode, set_point, speed):
        """调用基础运动服务"""
        req = BaseMotion.Request()
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
            case BaseMotionMode.LINE:
                # 为什么有 rotate ?
                req.rotate_param.kp = 1.8
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = float(speed)
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.8
                req.rotate_param.ek = 1.0
                req.rotate_param.steady_clk = 2
            case BaseMotionMode.ROTATE:
                req.rotate_param.kp = 2.8
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = float(speed)
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.7
                req.rotate_param.ek = 1.0
                req.rotate_param.steady_clk = 10

        return self.__motion_srv.call_async(req)

    def base_motion_line(self, distance: float, speed: float, is_block=True):
        """基础运动: 直线模式"""
        self.__logger.info(f'[基础运动] 直线运动 距离 {distance} 速度 {speed}')

        future = self.__call_srv_base_motion(BaseMotionMode.LINE, distance, speed)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            if not future.result().success:
                self.__logger.error('[基础运动] 错误, 无法直线运动!')
                return
            break

        if is_block:
            self.wait_base_motion()

    def base_motion_rotate(self, angle: float, speed: float, is_block=True):
        """基础运动: 旋转模式"""
        self.__logger.info(f'[基础运动] 旋转运动 角度 {angle} 速度 {speed}')

        future = self.__call_srv_base_motion(BaseMotionMode.ROTATE, angle, speed)
        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            if not future.result().success:
                self.__logger.error('[基础运动] 错误, 无法旋转运动!')
                return
            break

        if is_block:
            self.wait_base_motion()

    def wait_base_motion(self):
        """
        等待基础运动完成
        {'feedback': {'motion_mode': 0, 'motion_status': 2, 'error': 0.0}, 'success': True}
        motion_mode: 0:空闲 1:直线 2:旋转
        motion_status: 0:空闲 1:运行中 2:完成
        """
        flag = True
        while flag:
            future = self.__call_srv_base_motion(BaseMotionMode.QUERY, 0, 0)

            # 这里鉴于非同步式的特殊性质，需要双层while循环请求并验证是否有效
            while rclpy.ok():
                rclpy.spin_once(self.__node)

                if not future.done():
                    continue

                feedback = future.result().feedback
                if feedback.motion_mode == 0 or feedback.motion_status == 2 :
                    self.__logger.info("[基础运动] 运动完成")
                    flag = False
                    break
                else:
                    break

    def stop_base_motion(self):
        """停止基础运动"""
        self.__logger.info("[基础运动] 停止中...")

        future = self.__call_srv_base_motion(BaseMotionMode.STOP, 0, 0)
        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            if future.result().success:
                self.__logger.info('[基础运动] 停止完成.')
            else:
                self.__logger.error('[基础运动] 停止失败.')
