import rclpy
import rclpy.action

from corn_robot_interfaces.action import NavigationToWaypoint
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MyNavigationDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()
        self.__is_navigating = False

        self.__action = rclpy.action.ActionClient(node, NavigationToWaypoint, '/navigation_to_waypoint')

    def navigation(self, waypoint_name: str, speed: float) -> None:
        """

        """
        goal_msg = NavigationToWaypoint.Goal()

        goal_msg.waypoint_name = waypoint_name
        goal_msg.speed = speed

        self.__action.wait_for_server()
        self.__logger.info(f"正在发送新的导航请求")

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
