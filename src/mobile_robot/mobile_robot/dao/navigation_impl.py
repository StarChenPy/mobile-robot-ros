"""
    导航接口
        1: path_follow 路径跟随
        2: wait_finished_path_follow 等待路径跟随结束
        3: get_path_follow_status 路径跟随导航状态
        4: cancel_path_follow: 取消路径跟随
        5: get_robot_pose: 获取当前pose(预留接口)
        6: base_motion: 基础运动
        7: stop_base_motion: 停止基础运动
        8: base_motion_feedback: 基础运动状态反馈读取
        9: wait_base_motion: 等待基础运动完成

    Version:        V1.0
    Author:         Yongjie.Hu
    Revised Date:   2024/12/19
                    com.huibo.robot
"""
import math
import time
import rclpy
from rclpy.action import ActionClient
from base_nav2.action import NavCMD
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.srv import SetMap
from base_motion_ros2.srv import BaseMotion
import threading
from base_nav2_ptp.action import NavPTPCMD


# from robot_manager.msg import PathPointImpl
# from robot_manager.msg import PoseInit

class NavigationImpl:
    __ros = None
    # base_nav2接口
    __action_base_nav2 = None  # Action客户端
    __goal_handle_base_nav2 = None
    __send_goal_future_base_nav2 = None  # Goal 请求
    __get_result_future_base_nav2 = None  # Goal result
    __is_naving = False  # 正在导航中
    __srv_base_motion = None

    if_is_naving = False  # 外部调用

    __mtx_base_nav2 = threading.Lock()  # base_nav2 互斥锁

    def __init__(self, ros):
        self.__ros = ros
        # 路径点接口服务
        self.__srv_base_motion = self.__ros.create_client(BaseMotion,
                                                          '/base_motion')  # 'robot_manager/PathPointImpl'
        # movebase action
        self.__action_base_nav2 = ActionClient(self.__ros, NavCMD, '/nav2_action')  # , 'move_base_msgs/MoveBaseAction'
        self.__action_base_nav2_ptp = ActionClient(self.__ros, NavPTPCMD, '/nav2_ptp_action')

        print('[导航接口] 初始化完成.')

    # 点至点坐标驱动器(路径坐标[xy], 最终角度deg, 倒车, 最大线速度0.5m/s, 最大角速度3.0rad/s, 直线加速度, 直线减加速度，旋转加速度3.0, 旋转减加速度3.0)
    def point_driver(self, points=[], heading=0.0, back=False, linear_vel=0.5, angular_vel=3.0,
                     linear_acc=10.0, linear_decel=10.0, rotate_acc=3.0, rotate_decel=3.0):
        goal_msg = NavPTPCMD.Goal()
        # 载入goal_msg参数
        for pt in points:
            pose2d = Pose2D()
            pose2d.x = float(pt['x'])
            pose2d.y = float(pt['y'])
            pose2d.theta = 0.0
            goal_msg.points.append(pose2d)  # 路径点xy
        goal_msg.heading = float(heading)  # 最终航向
        goal_msg.back = bool(back)  # 倒车
        goal_msg.linear_vel = float(linear_vel)  # 最大线速度
        goal_msg.rotation_vel = float(angular_vel)  # 最大角速度
        goal_msg.linear_acc = float(linear_acc)  # 直线加速度
        goal_msg.linear_decel = float(linear_decel)  # 直线减加速度
        goal_msg.rotate_acc = float(rotate_acc)  # 旋转加速度
        goal_msg.rotate_decel = float(rotate_decel)  # 旋转减加速度
        # 等待action服务器连接
        self.__action_base_nav2_ptp.wait_for_server()
        print("[导航接口] base_nav2_ptp 正在发送新的导航请求:\n", goal_msg)
        # 创建action的句柄
        self.__send_goal_future_base_nav2_ptp = self.__action_base_nav2_ptp.send_goal_async(goal_msg)
        self.__send_goal_future_base_nav2_ptp.add_done_callback(self.__goal_response_callback_base_nav2_ptp)
        self.__mtx_base_nav2.acquire()
        self.__is_naving = True
        self.if_is_naving = True
        self.__mtx_base_nav2.release()

    # 等待点至点坐标驱动器导航结束
    def wait_finished_point_driver(self):
        print("[导航接口] base_nav2_ptp 等待导航结束中...")
        while True:
            rclpy.spin_once(self.__ros)
            self.__mtx_base_nav2.acquire()
            # 导航中为False时导航完成
            if self.__is_naving == False:
                self.__mtx_base_nav2.release()
                break
            self.__mtx_base_nav2.release()
            time.sleep(0.1)
        print("[导航接口] base_nav2_ptp 导航结束...")
        return

    # 获取点至点坐标驱动器导航状态 True:正在导航
    def get_point_driver_status(self):
        state = False
        self.__mtx_base_nav2.acquire()
        state = self.__is_naving
        self.__mtx_base_nav2.release()
        return state

    # 取消点至点坐标驱动器导航
    def cancel_point_driver(self):
        print("[导航接口] 取消导航...")
        if self.__goal_handle_base_nav2_ptp is not None:
            cancel_future = self.__goal_handle_base_nav2_ptp.cancel_goal_async()

    def __goal_response_callback_base_nav2_ptp(self, future):
        self.__goal_handle_base_nav2_ptp = future.result()
        if not self.__goal_handle_base_nav2_ptp.accepted:
            # Action服务器正在导航中...无法执行新Goal
            print("[导航接口] base_nav2_ptp 错误!服务端拒绝本次Goal请求!")
            return
        print("[导航接口] base_nav2_ptp 正在执行导航...")
        # 注册Goal回调
        self.__get_result_future_base_nav2_ptp = self.__goal_handle_base_nav2_ptp.get_result_async()
        self.__get_result_future_base_nav2_ptp.add_done_callback(self.__get_result_callback_ptp)

    def __get_result_callback_ptp(self, future):
        nav_result = future.result().result
        print("[导航接口] base_nav2_ptp 导航完成: ", nav_result)
        # 互斥锁
        self.__mtx_base_nav2.acquire()
        self.__is_naving = False
        self.if_is_naving = False
        self.__mtx_base_nav2.release()

    # 路径跟随(路径坐标[xy], 最终角度deg, 倒车, 最大线速度0.5m/s, 最大角速度3.0rad/s, 旋转加速度3.0, 旋转减加速度3.0)
    def path_follow(self, points=[], heading=0.0, back=False, linear_vel=0.5, angular_vel=3.0, rotate_acc=3.0,
                    rotate_decel=3.0):
        goal_msg = NavCMD.Goal()
        # 载入goal_msg参数
        for pt in points:
            pose2d = Pose2D()
            pose2d.x = float(pt['x'])
            pose2d.y = float(pt['y'])
            pose2d.theta = 0.0
            goal_msg.points.append(pose2d)  # 路径点xy
        goal_msg.heading = float(heading)  # 最终航向
        goal_msg.back = bool(back)  # 倒车
        goal_msg.linear_vel = float(linear_vel)  # 最大线速度
        goal_msg.rotation_vel = float(angular_vel)  # 最大角速度
        goal_msg.rotate_acc = float(rotate_acc)  # 旋转加速度
        goal_msg.rotate_decel = float(rotate_decel)  # 旋转减加速度
        # 等待action服务器连接
        self.__action_base_nav2.wait_for_server()
        print("[导航接口] base_nav2 正在发送新的导航请求:\n", goal_msg)
        # 创建action的句柄
        self.__send_goal_future_base_nav2 = self.__action_base_nav2.send_goal_async(goal_msg)
        self.__send_goal_future_base_nav2.add_done_callback(self.__goal_response_callback_base_nav2)
        self.__mtx_base_nav2.acquire()
        self.__is_naving = True
        self.if_is_naving = True
        self.__mtx_base_nav2.release()

    # base_nav2 Goal接受回调
    def __goal_response_callback_base_nav2(self, future):
        self.__goal_handle_base_nav2 = future.result()
        if not self.__goal_handle_base_nav2.accepted:
            # Action服务器正在导航中...无法执行新Goal
            print("[导航接口] base_nav2 错误!服务端拒绝本次Goal请求!")
            return
        print("[导航接口] base_nav2 正在执行导航...")
        # 注册Goal回调
        self.__get_result_future_base_nav2 = self.__goal_handle_base_nav2.get_result_async()
        self.__get_result_future_base_nav2.add_done_callback(self.__get_result_callback)

    # base_nav2 Goal Result回调
    def __get_result_callback(self, future):
        nav_result = future.result().result
        print("[导航接口] base_nav2 导航完成: ", nav_result)
        # 互斥锁
        self.__mtx_base_nav2.acquire()
        self.__is_naving = False
        self.if_is_naving = False
        self.__mtx_base_nav2.release()

    # 等待路径跟随导航结束
    def wait_finished_path_follow(self):
        print("[导航接口] base_nav2 等待导航结束中...")
        while True:
            self.__mtx_base_nav2.acquire()
            # 导航中为False时导航完成
            if self.__is_naving == False:
                self.__mtx_base_nav2.release()
                break
            self.__mtx_base_nav2.release()
            time.sleep(0.1)
        print("[导航接口] base_nav2 导航结束...")
        return

    # 获取导航状态 True:正在导航
    def get_path_follow_status(self):
        state = False
        self.__mtx_base_nav2.acquire()
        state = self.__is_naving
        self.__mtx_base_nav2.release()
        return state

    # 取消base nav2导航
    def cancel_path_follow(self):
        print("[导航接口] 取消导航...")
        if self.__goal_handle_base_nav2 is not None:
            cancel_future = self.__goal_handle_base_nav2.cancel_goal_async()

    def __call_srv_base_motion(self, motion_mode, set_point, max_vel):
        # 直线参数
        req = BaseMotion.Request()
        req.motion_mode = motion_mode
        req.set_point = float(set_point)
        req.line_param.kp = 2.3
        req.line_param.ti = 0.0
        req.line_param.td = 0.0
        req.line_param.max_vel = 1.0
        req.line_param.max_acc = 5000.0
        req.line_param.low_pass = 0.9
        req.line_param.ek = 0.01
        req.line_param.steady_clk = 5

        # 规划器参数 直线模式
        req.line_param.planner_vel = float(max_vel)  # 速度
        req.line_param.planner_acc = 1000.0  # 加速度
        req.line_param.planner_decel = 1000.0  # 减加速度

        if motion_mode == 2:
            # 参数 直线模式
            req.rotate_param.kp = 2.6
            req.rotate_param.ti = 0.0
            req.rotate_param.td = 0.0000
            req.rotate_param.max_vel = float(180.0)
            req.rotate_param.max_acc = 1500.0
            req.rotate_param.low_pass = 1.0
            req.rotate_param.ek = 1.0
            req.rotate_param.steady_clk = 2

        elif motion_mode == 3:
            # 参数 旋转模式
            req.rotate_param.kp = 2.6
            req.rotate_param.ti = 0.0
            req.rotate_param.td = 0.0000
            req.rotate_param.max_vel = 3.5 * 180.0 / math.pi
            req.rotate_param.max_acc = 1500.0
            req.rotate_param.low_pass = 0.9
            req.rotate_param.ek = 1.0
            req.rotate_param.steady_clk = 10

            # 规划器参数 旋转模式
            req.rotate_param.planner_vel = float(max_vel)  # 速度
            req.rotate_param.planner_acc = 600.0  # 加速度
            req.rotate_param.planner_decel = 300.0  # 减加速度

        while not self.__srv_base_motion.wait_for_service(timeout_sec=1):
            print('BaseMotion 等待服务中...')
        res = self.__srv_base_motion.call(req)
        return res

        # base motion 旋转

    def base_motion(self, mode='rotate', set_deg=0.0, max_vel=0.0):
        print('[基础运动] {}. 设定值:{} 速度:{}'.format(mode, set_deg, max_vel))
        mode_tbl = {'line': 2, 'rotate': 3}
        res = self.__call_srv_base_motion(mode_tbl[mode], set_deg, max_vel)
        success = res.success
        if not success:
            print('[基础运动] 错误, 无法启动运动!', mode)

        # time.sleep(0.8)
        return success

    # 停止基础运动
    def stop_base_motion(self):
        time.sleep(0.8)
        print("start")
        res = self.__call_srv_base_motion(1, 0, 0)
        if res.success:
            print('[基础运动] 停止完成.')
        else:
            print('[基础运动] 停止失败.')
        return res.success

    # 获取基础运动反馈
    def base_motion_feedback(self):
        res = self.__call_srv_base_motion(0, 0, 0)
        return res

    # 等待基础运动完成
    def wait_base_motion(self):
        """
           {'feedback': {'motion_mode': 0, 'motion_status': 2, 'error': 0.0}, 'success': True}
            motion_mode: 0:空闲 1:直线 2:旋转
            motion_status: 0:空闲 1:运行中 2:完成
        """

        while True:
            time.sleep(0.2)
            res = self.__call_srv_base_motion(0, 0, 0)
            # print(res)
            if res.feedback.motion_mode == 0 or res.feedback.motion_status == 2:
                break
