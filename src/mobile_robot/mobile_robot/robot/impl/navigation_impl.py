import time
import math

from enum import Enum
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.action import ActionClient
from .io_impl import IoImpl
from base_motion_ros2.srv import BaseMotion
from base_nav2.action import NavCMD
from chassis_msgs.srv import ResetOdom


class ResetOdomMode(Enum):
    RESET_ALL = 0
    RESET_POSE = 1
    RESET_YAW = 2


class BaseMotionMode(Enum):
    QUERY = 0
    STOP = 1
    LINE = 2
    ROTATE = 3


class NavigationImpl:
    __goal_handle = None

    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        self.__io = IoImpl.instance(node)

        self.__motion_srv = node.create_client(BaseMotion, '/base_motion')
        self.__odom_srv = node.create_client(ResetOdom, '/chassis/reset_odom')
        self.__navigation_action = ActionClient(node, NavCMD, '/nav2_action')

        self.__logger.info("[导航接口] 初始化完成.")

    # ===============================导航部分===============================

    def init_pose(self, x=0.0, y=0.0, angle=0.0, mode=ResetOdomMode.RESET_ALL):
        """初始化机器人位置，支持重置odom不同模式"""
        self.__logger.info(f"[导航接口] 初始化机器人位置 [{x}, {y}, {angle}] 模式为 {mode.name}")

        req = ResetOdom.Request()
        req.clear_mode = mode.value
        req.x = float(x)
        req.y = float(y)

        radian = math.radians(angle)
        req.theta = float(radian)
        self.__logger.info(f'输入角度 {angle} 转弧度 {radian}')

        res = self.__odom_srv.call(req)

        if res.success:
            self.__logger.info("[导航接口] 重置 Odometry 成功")
            return True
        else:
            self.__logger.error("[导航接口] 重置 Odometry 错误")
            return False

    def navigation(self, points: tuple, heading=0.0, reverse=False, linear_speed=0.55, rotation_speed=3.5):
        """
        路径跟随: 输入路径点、最终角度等参数，发送导航请求
        @param points 路径坐标点[x,y]
        @param heading 最终点航向
        @param reverse 倒车
        @param linear_speed 最大线速度m/s
        @param rotation_speed 最大旋转速度m/s
        """

        goal_msg = NavCMD.Goal()

        for p in points:
            pose2d = Pose2D()
            pose2d.x = float(p['x'])
            pose2d.y = float(p['y'])
            pose2d.theta = 0.0
            goal_msg.points.append(pose2d)

        goal_msg.heading = float(heading)
        goal_msg.back = bool(reverse)
        goal_msg.linear_vel = float(linear_speed)
        goal_msg.rotation_vel = float(rotation_speed)

        self.__logger.info("[导航接口] 等待导航服务")
        self.__navigation_action.wait_for_server()
        self.__logger.info("[导航接口] base_nav2 正在发送新的导航请求")

        __send_goal_future = self.__navigation_action.send_goal_async(goal_msg)
        __send_goal_future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future):
        """处理Goal响应"""
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.__logger.error("[导航接口] base_nav2 错误!服务端拒绝本次Goal请求!")
            return

        self.__logger.info("[导航接口] base_nav2 正在执行导航...")
        __get_result_future = self.__goal_handle.get_result_async()
        __get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, future):
        """处理Goal完成回调"""
        nav_result = future.result().result
        self.__logger.info(f"[导航接口] base_nav2 导航完成: {nav_result}")

    def wait_navigation_finish(self):
        """等待导航完成"""
        self.__logger.info("[导航接口] base_nav2 等待导航结束中...")
        while not self.__goal_handle.done():
            pass
        self.__logger.info("[导航接口] base_nav2 导航结束.")

    def cancel_navigation(self):
        """取消路径跟随"""
        self.__logger.info("[导航接口] 取消导航...")
        if self.__goal_handle:
            self.__goal_handle.cancel_goal_async()

    # ===============================基础运动部分===============================

    def __call_srv_base_motion(self, mode: BaseMotionMode, set_point, speed):
        """调用基础运动服务"""
        req = BaseMotion.Request()
        req.motion_mode = mode.value
        req.set_point = float(set_point)

        match mode:
            case BaseMotionMode.LINE:
                req.line_param.kp = 1.8
                req.line_param.ti = 0.0
                req.line_param.td = 0.0
                req.line_param.max_vel = float(speed)
                req.line_param.max_acc = 3.0
                req.line_param.low_pass = 0.8
                req.line_param.ek = 0.02
                req.line_param.steady_clk = 5
            case BaseMotionMode.ROTATE:
                req.rotate_param.kp = 2.8
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = float(speed)
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.7
                req.rotate_param.ek = 1.0
                req.rotate_param.steady_clk = 10

        return self.__motion_srv.call(req)

    def base_motion_line(self, distance, speed=10):
        """基础运动: 直线或旋转模式"""
        self.__logger.info(f'[基础运动] 直线运动 距离 {distance} 速度 {speed}')

        res = self.__call_srv_base_motion(BaseMotionMode.LINE, distance, speed)
        if not res.success:
            self.__logger.error('[基础运动] 错误, 无法直线运动!')
            return False

        return True

    def base_motion_rotate(self, angle=0.0, speed=0.0):
        """基础运动: 直线或旋转模式"""
        self.__logger.info(f'[基础运动] 旋转运动 角度 {angle} 速度 {speed}')

        res = self.__call_srv_base_motion(BaseMotionMode.ROTATE, angle, speed)
        if not res.success:
            self.__logger.error('[基础运动] 错误, 无法启动运动!')
            return False

        return True

    def stop_base_motion(self):
        """停止基础运动"""
        self.__logger.info("[基础运动] 停止中...")
        res = self.__call_srv_base_motion(BaseMotionMode.STOP, 0, 0)
        if res.success:
            self.__logger.info('[基础运动] 停止完成.')
        else:
            self.__logger.error('[基础运动] 停止失败.')
        return res.success
