import geometry_msgs.msg
import rclpy
import rclpy.action

from base_nav2_ptp.action import NavPTPCMD
from ..popo.NavigationPoint import NavigationPoint
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class NavigationPtpDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()
        self.__is_navigating = False

        self.__action = rclpy.action.ActionClient(self.__node, NavPTPCMD, '/nav2_ptp_action')

    def navigation(self, points: list[NavigationPoint], linear_speed: float, rotation_speed: float, reverse: bool):
        """
        路径跟随: 输入路径点、最终角度等参数，发送导航请求
        @param points 路径坐标点 NavigationPoint(x, y, yaw)
        @param linear_speed 最大线速度m/s
        @param rotation_speed 最大旋转速度m/s
        @param linear_acc 最大线加速度
        @param rotate_acc 最大旋转减速度
        @param reverse 倒车模式
        """
        goal_msg = NavPTPCMD.Goal()

        for p in points:
            pose2d = geometry_msgs.msg.Pose2D(x=float(p.x), y=float(p.y), theta=float(0))
            goal_msg.points.append(pose2d)

        # 这里要获取导航最后一个点的角度并赋给heading
        # goal_msg.linear_vel = float(linear_speed)
        # goal_msg.rotation_vel = float(rotation_speed)
        # goal_msg.linear_acc = float(linear_acc)  # 直线加速度
        # goal_msg.linear_decel = float(linear_acc)  # 直线减加速度
        # goal_msg.rotate_acc = float(rotate_acc)  # 旋转加速度
        # goal_msg.rotate_decel = float(rotate_acc)  # 旋转减加速度
        goal_msg.linear_vel = float(linear_speed)
        goal_msg.rotation_vel = float(rotation_speed)
        goal_msg.linear_acc = float(0.69)  # 直线加速度
        goal_msg.linear_decel = float(0.33)  # 直线减加速度
        goal_msg.rotate_acc = float(2.5)  # 旋转加速度
        goal_msg.rotate_decel = float(1)  # 旋转减加速度
        goal_msg.heading = float(points[-1].yaw)
        goal_msg.back = reverse

        self.__action.wait_for_server()
        self.__logger.info(f"正在发送新的导航请求: {points}, 速度: {linear_speed}")

        goal_handle = self.__action.send_goal_async(goal_msg)
        goal_handle.add_done_callback(self.__goal_response_callback)

        self.__is_navigating = True

    def __goal_response_callback(self, future) -> None:
        """处理Goal响应"""
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.__logger.error("服务端拒绝本次请求!")
            return

        self.__logger.info("正在执行导航...")
        __get_result_future = self.__goal_handle.get_result_async()
        __get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, _) -> None:
        """处理Goal完成回调"""
        self.__logger.info(f"导航完成")

        self.__is_navigating = False

    def wait_finish(self) -> None:
        """等待导航完成"""
        while rclpy.ok():
            rclpy.spin_once(self.__node)
            if not self.__is_navigating:
                break
        self.__logger.info("导航结束")

    def cancel(self) -> None:
        """取消路径跟随"""
        while self.__goal_handle is None:
            rclpy.spin_once(self.__node)

        assert self.__goal_handle is not None
        self.__logger.info("取消导航")
        self.__goal_handle.cancel_goal_async()

    def get_status(self) -> bool:
        """
        获取导航状态
        @return True 正在导航
        """
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        status = self.__is_navigating
        return status
