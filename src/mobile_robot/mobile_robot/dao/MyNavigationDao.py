import asyncio

import rclpy
import rclpy.action
from rclpy.task import Future

from corn_robot_interfaces.action import NavigationToWaypoint
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MyNavigationDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()
        self.__is_navigating = False
        self.__goal_handle = None
        self.__done_future: Future | None = None

        self.__action = rclpy.action.ActionClient(
            node, NavigationToWaypoint, '/navigation_to_waypoint'
        )

    def navigation(self, waypoint_name: str, speed: float, start_name: str = "") -> None:
        goal_msg = NavigationToWaypoint.Goal()
        goal_msg.goal_name = waypoint_name
        goal_msg.start_name = start_name
        goal_msg.speed = speed

        self.__action.wait_for_server()
        self.__logger.info(f"正在发送新的导航请求")

        goal_handle_future = self.__action.send_goal_async(goal_msg)
        goal_handle_future.add_done_callback(self.__goal_response_callback)

        self.__is_navigating = True
        self.__done_future = Future()

    def __goal_response_callback(self, future) -> None:
        """处理Goal响应"""
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.__logger.error("服务端拒绝本次请求!")
            self.__is_navigating = False
            if self.__done_future is not None:
                self.__done_future.set_result(False)
            return

        self.__logger.info("正在执行导航...")
        result_future = self.__goal_handle.get_result_async()
        result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, _) -> None:
        """处理Goal完成回调"""
        self.__logger.info(f"导航完成")
        self.__is_navigating = False
        if self.__done_future is not None:
            self.__done_future.set_result(True)

    def wait_finish(self) -> None:
        """等待导航完成（同步接口，但内部用 asyncio 轮询）"""
        loop = asyncio.get_event_loop()

        async def _wait():
            while rclpy.ok() and self.__is_navigating:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
                await asyncio.sleep(0.5)  # 让出控制权
            self.__logger.info("导航结束")

        # 在当前事件循环中运行协程，直到完成
        loop.run_until_complete(_wait())

    def cancel(self) -> None:
        """取消路径跟随"""
        while self.__goal_handle is None:
            rclpy.spin_once(self.__node)

        assert self.__goal_handle is not None
        self.__logger.info("取消导航")
        self.__goal_handle.cancel_goal_async()

    def get_status(self) -> bool:
        """获取导航状态"""
        rclpy.spin_once(self.__node, timeout_sec=0.1)
        return self.__is_navigating
