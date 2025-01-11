import time
import math

from rclpy.node import Node
from rclpy.action import ActionClient

from .io_impl import IoImpl
from base_motion_ros2.srv import BaseMotion
from base_nav.action import NavigationActionCMD
from chassis_msgs.srv import ResetOdom


class NavigationImpl:
    __goal_handle = None
    __navigating = False

    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        self.__io = IoImpl.instance(node)

        self.__motion_srv = node.create_client(BaseMotion, '/base_motion')
        self.__odom_srv = node.create_client(ResetOdom, '/chassis/reset_odom') #, 'chassis_msgs/ResetOdom'
        self.__navigation_action = ActionClient(node, NavigationActionCMD, '/nav_action')

        self.__logger.info("[导航接口] 初始化完成.")

    #设置odom mode:0重置[x y theta] 1重置[x y] 2重置[theta]
    def init_pose(self, x=0.0, y=0.0, w=0.0, mode=0):
        print("[底盘接口] 初始化机器人位置 [{} {} {}]".format(x, y, w))

        req = ResetOdom.Request()
        req.clear_mode = 0
        req.x = float(x)
        req.y = float(y)
        # W 角度转弧度
        if mode !=0 :
            pass
        else:
            angle = math.radians(w)
            radian = angle * 180.0 / math.pi
            self.__logger.info(f'输入角度转弧度: {angle}, 弧度转回角度{radian}')
            req.theta = float(angle)

        res = self.__odom_srv.call(req)

        if res.success:
            self.__logger.info("[底盘接口] 重置 Odometry 成功")
            return True
        else:
            self.__logger.error("[底盘接口] 重置 Odometry 错误")
            return False

    #导航到pose
    def move_to_point(self, x, y, w, is_rotate = False, speed = 0.2):
        goal_msg = NavigationActionCMD.Goal()
        goal_msg.target_x = float(x)
        goal_msg.target_y = float(y)
        goal_msg.target_theta = float(w)
        goal_msg.is_rotation = is_rotate
        goal_msg.param.max_vel = speed

        self.__navigation_action.wait_for_server()

        self.__logger.info("[导航接口] 正在发送新的导航请求[{} {} {}]".format(x, y, w))

        # 创建 action 的句柄
        send_goal_future = self.__navigation_action.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, goal_handle):
        self.__goal_handle = goal_handle.result()
        if not self.__goal_handle.accepted:
            self.__logger.info('[导航模块] 拒绝目标点')
            return

        self.__navigating = True
        self.__logger.info('[导航模块] 正在执行目标点')

        _get_result_future = self.__goal_handle.get_result_async()
        _get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, goal_handle):
        nav_result = goal_handle.result().result
        self.__navigating = False

        self.__logger.info("[导航接口] 执行完毕: ",nav_result.message)

    def point_follow(self, points: list, max_vel = 0.2):
        #导航
        for i, point in enumerate(points):
            self.__logger.info("[导航接口] 正在导航至目标点:", points[i])

            is_rotate = False
            # 最后一个点需要主动执行最后的yaw
            if i == points.__len__() - 1:
                is_rotate = True

            self.move_to_point(point['x'], point['y'], point['w'], is_rotate, max_vel)
            self.wait_finished()

    #等待导航结束
    def wait_finished(self):
        self.__logger.info("[导航接口] 等待导航结束中...")

        while self.__navigating:
            time.sleep(0.2)
        self.__logger.info("[导航接口] 导航结束...")

    # 取消导航
    def cancel_navigation(self):
        self.__goal_handle.cancel_goal_async()

    # 基础运动服务
    def __call_motion_srv(self, motion_mode, set_point, max_vel):
        req = BaseMotion.Request()
        req.motion_mode = motion_mode
        req.set_point = float(set_point)
        req.line_param.kp = 1.5
        req.line_param.ti = 0.0
        req.line_param.td = 0.0
        req.line_param.max_vel = float(max_vel)
        req.line_param.max_acc = 3.0
        req.line_param.low_pass = 0.8
        req.line_param.ek = 0.01
        req.line_param.steady_clk = 5

        match motion_mode:
            case 2:
                # 参数 直线运动模式
                req.rotate_param.kp = 2.3
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = float(max_vel)
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.8
                req.rotate_param.ek = 5.0
                req.rotate_param.steady_clk = 5
            case 3:
                #参数 旋转运动模式
                req.rotate_param.kp = 1.8
                req.rotate_param.ti = 0.0
                req.rotate_param.td = 0.0001
                req.rotate_param.max_vel = float(max_vel)
                req.rotate_param.max_acc = 600.0
                req.rotate_param.low_pass = 0.7
                req.rotate_param.ek = 1.0
                req.rotate_param.steady_clk = 10

        res = self.__motion_srv.call(req)
        return res

    # 旋转
    def rotate_motion(self, angle, speed):
        res = self.__call_motion_srv(3, angle, speed)
        success = res.success
        if not success:
            self.__logger.info('[基础运动] 错误, 无法旋转!')
        return success

    # 直行
    def linear_motion(self, distance, speed):
        res = self.__call_motion_srv(2, distance, speed)
        success = res.success
        if not success:
            self.__logger.info('[基础运动] 错误, 无法直线运动!')
        return success

    # 停止基础运动
    def stop_motion(self):
        res = self.__call_motion_srv(1, 0, 0)
        if res.success:
            self.__logger.info('[基础运动] 停止完成.')
        else:
            self.__logger.info('[基础运动] 停止失败.')
        return res.success

    # 获取基础运动反馈
    def motion_feedback(self):
        res = self.__call_motion_srv(0, 0, 0)
        return res

    # 等待基础运动完成
    def wait_motion(self):
        """
           {'feedback': {'motion_mode': 0, 'motion_status': 2, 'error': 0.0}, 'success': True}
            motion_mode: 0:空闲 1:直线 2:旋转
            motion_status: 0:空闲 1:运行中 2:完成
        """
        while True:
            res = self.__call_motion_srv(0, 0, 0)
            if res.feedback.motion_mode == 0 or res.feedback.motion_status == 2 :
                break
            time.sleep(0.2)