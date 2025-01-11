import time
import math
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.action import ActionClient
from .io_impl import IoImpl
from base_motion_ros2.srv import BaseMotion
from base_nav2.action import NavCMD
from chassis_msgs.srv import ResetOdom


class NavigationImpl:
    __send_goal_future = None
    __goal_handle = None
    __navigating = False

    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        self.__io = IoImpl.instance(node)

        self.__motion_srv = node.create_client(BaseMotion, '/base_motion')
        self.__odom_srv = node.create_client(ResetOdom, '/chassis/reset_odom')
        self.__navigation_action = ActionClient(node, NavCMD, '/nav2_action')

        self.__logger.info("[导航接口] 初始化完成.")

    def init_pose(self, x=0.0, y=0.0, w=0.0, mode=0):
        """初始化机器人位置，支持重置odom不同模式"""
        self.__logger.info("[导航接口] 初始化机器人位置 [{} {} {}]".format(x, y, w))

        req = ResetOdom.Request()
        req.clear_mode = mode
        req.x = float(x)
        req.y = float(y)

        if mode == 0:
            angle = math.radians(w)
            req.theta = float(angle)
            self.__logger.info(f'输入角度转弧度: {angle}')

        res = self.__odom_srv.call(req)

        if res.success:
            self.__logger.info("[导航接口] 重置 Odometry 成功")
            return True
        else:
            self.__logger.error("[导航接口] 重置 Odometry 错误")
            return False

    def path_follow(self, points=[], heading=0.0, back=False, linear_vel=0.55, angular_vel=3.5):
        """路径跟随: 输入路径点、最终角度等参数，发送导航请求"""
        goal_msg = NavCMD.Goal()

        for pt in points:
            pose2d = Pose2D()
            pose2d.x = float(pt['x'])
            pose2d.y = float(pt['y'])
            pose2d.theta = 0.0
            goal_msg.points.append(pose2d)

        goal_msg.heading = float(heading)
        goal_msg.back = bool(back)
        goal_msg.linear_vel = float(linear_vel)
        goal_msg.rotation_vel = float(angular_vel)

        self.__logger.info("[导航接口] 等待导航服务")
        self.__navigation_action.wait_for_server()
        self.__logger.info("[导航接口] base_nav2 正在发送新的导航请求")

        self.__send_goal_future = self.__navigation_action.send_goal_async(goal_msg)
        self.__send_goal_future.add_done_callback(self.__goal_response_callback)

        self.__navigating = True

    def __goal_response_callback(self, future):
        """处理Goal响应"""
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.__logger.error("[导航接口] base_nav2 错误!服务端拒绝本次Goal请求!")
            return

        self.__logger.info("[导航接口] base_nav2 正在执行导航...")
        self.__get_result_future = self.__goal_handle.get_result_async()
        self.__get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, future):
        """处理Goal完成回调"""
        nav_result = future.result().result
        self.__logger.info(f"[导航接口] base_nav2 导航完成: {nav_result}")
        self.__navigating = False

    def wait_finished_path_follow(self):
        """等待路径跟随完成"""
        self.__logger.info("[导航接口] base_nav2 等待导航结束中...")
        while self.__navigating:
            time.sleep(0.1)
        self.__logger.info("[导航接口] base_nav2 导航结束.")

    def get_path_follow_status(self):
        """获取导航状态"""
        return self.__navigating

    def cancel_path_follow(self):
        """取消路径跟随"""
        self.__logger.info("[导航接口] 取消导航...")
        if self.__goal_handle:
            self.__goal_handle.cancel_goal_async()

    def base_motion(self, mode='rotate', set_deg=0.0, max_vel=0.0):
        """基础运动: 直线或旋转模式"""
        self.__logger.info(f'[基础运动] 模式: {mode}, 设定值: {set_deg}, 速度: {max_vel}')

        mode_tbl = {
            'line': 2,
            'rotate': 3
        }

        motion_mode = mode_tbl.get(mode)
        if motion_mode is None:
            self.__logger.error("[基础运动] 无效的模式!")
            return False

        res = self.__call_srv_base_motion(motion_mode, set_deg, max_vel)
        if not res.success:
            self.__logger.error('[基础运动] 错误, 无法启动运动!')
            return False

        return True

    def stop_base_motion(self):
        """停止基础运动"""
        time.sleep(0.8)
        self.__logger.info("[基础运动] 停止中...")
        res = self.__call_srv_base_motion(1, 0, 0)
        if res.success:
            self.__logger.info('[基础运动] 停止完成.')
        else:
            self.__logger.error('[基础运动] 停止失败.')
        return res.success

    def __call_srv_base_motion(self, motion_mode, set_point, max_vel):
        """调用基础运动服务"""
        req = BaseMotion.Request()
        req.motion_mode = motion_mode
        req.set_point = float(set_point)
        req.line_param.kp = 1.8
        req.line_param.ti = 0.0
        req.line_param.td = 0.0
        req.line_param.max_vel = float(max_vel)
        req.line_param.max_acc = 3.0
        req.line_param.low_pass = 0.8
        req.line_param.ek = 0.02
        req.line_param.steady_clk = 5

        if motion_mode == 3:  # Rotate mode specific params
            req.rotate_param.kp = 2.8
            req.rotate_param.ti = 0.0
            req.rotate_param.td = 0.0001
            req.rotate_param.max_vel = float(max_vel)
            req.rotate_param.max_acc = 600.0
            req.rotate_param.low_pass = 0.7
            req.rotate_param.ek = 1.0
            req.rotate_param.steady_clk = 10

        return self.__motion_srv.call(req)
