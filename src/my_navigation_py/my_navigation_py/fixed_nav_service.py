# import rclpy
# import math  # 新增：用于数学计算
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped
# from std_srvs.srv import Trigger
#
# class FixedNavService(Node):
#     def __init__(self):
#         super().__init__('fixed_nav_service')
#         self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#
#         # 创建服务
#         self._service = self.create_service(
#             Trigger,
#             'start_fixed_nav',
#             self.start_nav_callback
#         )
#
#         # 固定的导航目标点（可以根据需要修改这些值）
#         self.fixed_x = 0.7    # 固定X坐标
#         self.fixed_y = 3.7    # 固定Y坐标
#         self.fixed_yaw = 3.14  # 固定朝向（弧度）
#
#         self.get_logger().info(f"固定目标点导航服务已启动，目标点: x={self.fixed_x}, y={self.fixed_y}")
#
#         # 用于存储导航结果的变量
#         self._response_future = None
#
#     def start_nav_callback(self, request, response):
#         self.get_logger().info("收到导航请求，开始前往固定目标点...")
#
#         # 等待导航服务器就绪
#         if not self._action_client.wait_for_server(timeout_sec=5.0):
#             response.success = False
#             response.message = "导航服务器未启动，请先启动 Nav2！"
#             return response
#
#         # 构建目标位姿（使用固定目标点）
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = "map"
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#
#         goal_msg.pose.pose.position.x = self.fixed_x
#         goal_msg.pose.pose.position.y = self.fixed_y
#
#
#         q = self.quaternion_from_euler(0.0, 0.0, self.fixed_yaw)
#         goal_msg.pose.pose.orientation.x = q[0]
#         goal_msg.pose.pose.orientation.y = q[1]
#         goal_msg.pose.pose.orientation.z = q[2]
#         goal_msg.pose.pose.orientation.w = q[3]
#
#         # 发送目标并等待结果
#         self._send_goal_future = self._action_client.send_goal_async(
#             goal_msg, feedback_callback=self.feedback_callback
#         )
#         self._send_goal_future.add_done_callback(self.goal_response_callback)
#
#         # 等待导航完成
#         self._response_future = None
#         while rclpy.ok() and self._response_future is None:
#             rclpy.spin_once(self, timeout_sec=0.1)
#
#         if self._response_future is not None:
#             result = self._response_future.result().result
#             response.success = (result.status == 3)
#             response.message = f"导航结果: {result.status} (3表示成功)"
#         else:
#             response.success = False
#             response.message = "导航被中断"
#
#         return response
#
#     def quaternion_from_euler(self, roll, pitch, yaw):
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(pitch * 0.5)
#         sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll * 0.5)
#         sr = math.sin(roll * 0.5)
#
#         q = [0] * 4
#         q[0] = cy * cp * cr + sy * sp * sr  # w
#         q[1] = cy * cp * sr - sy * sp * cr  # x
#         q[2] = sy * cp * sr + cy * sp * cr  # y
#         q[3] = sy * cp * cr - cy * sp * sr  # z
#
#         return [q[1], q[2], q[3], q[0]]
#
#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info("导航目标被拒绝！")
#             self._response_future = None
#             return
#
#         self.get_logger().info("导航目标已接受，开始移动...")
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)
#
#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f"当前距离目标: {feedback.distance_remaining:.2f} 米")
#
#     def get_result_callback(self, future):
#         self._response_future = future
#         self.get_logger().info("导航任务完成")
#
# def main(args=None):
#     rclpy.init(args=args)
#     fixed_nav_service = FixedNavService()
#     rclpy.spin(fixed_nav_service)
#     fixed_nav_service.destroy_node()
#     rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()








#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import math
from typing import NamedTuple

# 定义返回结果的结构
class NavigationResult(NamedTuple):
    success: bool
    message: str


class FixedNavigationClient(Node):
    def __init__(self):
        super().__init__('fixed_nav_service')

        # 固定的导航目标点
        self.fixed_x = 0.7      # 固定X坐标
        self.fixed_y = 3.7     # 固定Y坐标
        self.fixed_yaw = 3.14   # 固定朝向（弧度）

        # 创建动作客户端
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def quaternion_from_yaw(self, yaw):
        """将偏航角（yaw）转换为四元数"""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def send_goal(self):
        """发送导航目标并等待结果"""
        # 等待动作服务器上线
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            return NavigationResult(success=False, message="导航动作服务器未响应")

        # 构造目标点
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.fixed_x
        goal_msg.pose.pose.position.y = self.fixed_y
        goal_msg.pose.pose.position.z = 0.0

        # 设置朝向（四元数）
        q = self.quaternion_from_yaw(self.fixed_yaw)
        goal_msg.pose.pose.orientation = q

        # 发送目标
        self.get_logger().info(
            f"发送导航目标: x={self.fixed_x}, y={self.fixed_y}, yaw={self.fixed_yaw}"
        )

        # 异步发送目标并等待结果
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle:
            return NavigationResult(success=False, message="目标发送失败")

        if not goal_handle.accepted:
            return NavigationResult(success=False, message="目标被拒绝")

        self.get_logger().info("目标已接受，等待导航完成...")

        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)

        if not result_future.done():
            return NavigationResult(success=False, message="等待结果超时")

        result = result_future.result().result
        status = result_future.result().status

        if status == 4:  # SUCCEEDED
            return NavigationResult(success=True, message="导航成功到达目标点")
        elif status == 5:  # CANCELED
            return NavigationResult(success=False, message="导航被取消")
        elif status == 6:  # ABORTED
            return NavigationResult(success=False, message="导航失败")
        else:
            return NavigationResult(success=False, message=f"导航未完成，状态码: {status}")


def call_fixed_navigation():
    """供外部脚本调用的导航函数"""
    # 注意：此处不初始化/关闭rclpy，由调用方处理
    node = FixedNavigationClient()
    try:
        result = node.send_goal()
    except Exception as e:
        result = NavigationResult(success=False, message=f"导航调用异常: {str(e)}")
    finally:
        node.destroy_node()
    return result



def main():
    print("导航服务启动中...")

if __name__ == '__main__':
    main()