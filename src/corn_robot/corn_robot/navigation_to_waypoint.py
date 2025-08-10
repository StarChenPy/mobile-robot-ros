import math
import time

import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D
from rclpy.action import ActionServer, ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from base_nav2_ptp.action import NavPTPCMD
from chassis_msgs.srv import ResetOdom
from corn_robot_interfaces.action import NavigationToWaypoint
from corn_robot_interfaces.msg import WaypointArray, Waypoint
from corn_robot_interfaces.srv import GeneratePath, CorrectionOdom
from corn_robot_toolbox.util import Math
from web_message_transform_ros2.msg import Pose, RobotData


class NavigationToPoseNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('navigation_to_waypoint')
        self.get_logger().info('路径点导航节点 正在初始化.')

        self.goal_handle = None
        self.waypoints = WaypointArray()
        self.odom = Pose()

        # 参数
        self.declare_parameter('robot_data_topic', '/web_transform_node/robot_data')
        robot_data_topic = self.get_parameter('robot_data_topic').value

        self.declare_parameter('set_odom_service', '/chassis/reset_odom')
        set_odom_service = self.get_parameter('set_odom_service').value

        self.declare_parameter('navigation_to_waypoint_action', '/navigation_to_waypoint')
        nav2wp_action = self.get_parameter('navigation_to_waypoint_action').value

        self.declare_parameter('generate_path_service', '/generate_path')
        generate_path_service = self.get_parameter('generate_path_service').value

        self.declare_parameter('correction_service', '/correction_odom')
        correction_service = self.get_parameter('correction_service').value

        self.declare_parameter('waypoints_topic', '/waypoints')
        waypoints_topic = self.get_parameter('waypoints_topic').value

        self.declare_parameter('navigation_action', '/nav2_ptp_action')
        navigation_action = self.get_parameter('navigation_action').value

        self.declare_parameter('speed', 0.4)
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('slope_offset', 0.35)

        # 缓存参数
        self.speed = self.get_parameter('speed').value

        # 通信接口
        self.action_server = ActionServer(self, NavigationToWaypoint, nav2wp_action,
                                          execute_callback=self.execute_callback,
                                          cancel_callback=self.cancel_callback)
        self.generate_path_client = self.create_client(GeneratePath, generate_path_service)
        self.correction_client = self.create_client(CorrectionOdom, correction_service)
        self.navigation_action = ActionClient(self, NavPTPCMD, navigation_action)
        self.set_odom_client = self.create_client(ResetOdom, set_odom_service)
        self.waypoints_sub = self.create_subscription(WaypointArray, waypoints_topic, self.waypoints_callback, 10)
        self.robot_data_sub = self.create_subscription(RobotData, robot_data_topic, self.robot_data_callback, 10)

        self.get_logger().info('路径点导航节点 已启动.')

    def waypoints_callback(self, msg: WaypointArray):
        self.waypoints = msg

    def robot_data_callback(self, msg: RobotData):
        self.odom = msg.odom

    def cancel_callback(self, _):
        self.get_logger().info('接收到取消导航到路径点请求.')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.get_logger().info('导航到路径点请求已取消.')
        return rclpy.action.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> NavigationToWaypoint.Result:
        goal: NavigationToWaypoint.Goal = goal_handle.request
        self.get_logger().info(f'接收到导航到路径点请求: {goal.waypoint_name}')
        T1 = time.process_time()

        self.speed = goal.speed if goal.speed > 0 else self.get_parameter('speed').value
        waypoints = await self.generate_path(goal.waypoint_name)
        if not waypoints:
            return self.create_result(False, '生成路径失败', -1, T1)

        path = []
        in_slope = False
        for index, waypoint in enumerate(waypoints):
            if len(waypoints) != 1 and index == 0:
                if waypoint.on_slope:
                    in_slope = True
                continue

            if goal_handle.status == GoalStatus.STATUS_CANCELING:
                self.get_logger().info("导航已被取消")
                goal_handle.canceled()
                return self.create_result(False, '导航已被取消', index, T1)

            appended = False

            # 判断是否为坡道点
            if waypoint.on_slope:
                if not in_slope:
                    self.get_logger().info(f"上坡，额外行走 {self.get_parameter('slope_offset').value} m")
                    in_slope = True

                    if waypoint.pose.w != 0:
                        w_back = waypoint.pose.w
                    else:
                        w_back = Math.compute_delta_theta(waypoint.pose, waypoints[index + 1].pose)
                    waypoint.pose.w = Math.compute_delta_theta(waypoints[index - 1].pose, waypoint.pose)
                    offset_pose = Math.get_target_coordinate(waypoint.pose, self.get_parameter('slope_offset').value)
                    offset_pose.w = w_back
                    waypoint.pose.w = w_back
                    if not appended:
                        appended = True
                        path.append(offset_pose)
                        if goal_handle.status == GoalStatus.STATUS_CANCELING:
                            self.get_logger().info("导航已被取消")
                            goal_handle.canceled()
                            return self.create_result(False, '导航已被取消', index, T1)
                        result = await self.navigation(path)
                        if result is None or not result.success:
                            goal_handle.abort()
                            return self.create_result(False, '导航到路径点失败', index, T1)
                        path = []

                    if goal_handle.status == GoalStatus.STATUS_CANCELING:
                        self.get_logger().info("导航已被取消")
                        goal_handle.canceled()
                        return self.create_result(False, '导航已被取消', index, T1)
                    request = ResetOdom.Request(clear_mode=0, x=waypoint.pose.x, y=waypoint.pose.y,
                                                theta=math.radians(waypoint.pose.w))
                    result = await self.set_odom_client.call_async(request)
                    if result is None or not result.success:
                        goal_handle.abort()
                        msg = result.message if result else '机器人设置Odom失败!'
                        return self.create_result(False, msg, index, T1)
            else:
                in_slope = False

            # 是否要倒车
            if len(waypoints) > 1:
                if Math.is_behind(waypoints[index - 1].pose, waypoint.pose, 170):
                    if not appended:
                        appended = True
                        path.append(waypoint.pose)
                        if goal_handle.status == GoalStatus.STATUS_CANCELING:
                            self.get_logger().info("导航已被取消")
                            goal_handle.canceled()
                            return self.create_result(False, '导航已被取消', index, T1)
                        result = await self.navigation(path, True)
                        if result is None or not result.success:
                            goal_handle.abort()
                            return self.create_result(False, '导航到路径点失败', index, T1)
                        path = []
            else:
                if Math.is_behind(self.odom, waypoint.pose, 170):
                    if not appended:
                        appended = True
                        path.append(waypoint.pose)
                        if goal_handle.status == GoalStatus.STATUS_CANCELING:
                            self.get_logger().info("导航已被取消")
                            goal_handle.canceled()
                            return self.create_result(False, '导航已被取消', index, T1)
                        result = await self.navigation(path, True)
                        if result is None or not result.success:
                            goal_handle.abort()
                            return self.create_result(False, '导航到路径点失败', index, T1)
                        path = []

            # 是否为矫正点
            corr_data = waypoint.corrected_data
            if corr_data.front or corr_data.back or corr_data.left or corr_data.right:
                if not appended:
                    appended = True
                    path.append(waypoint.pose)
                    if goal_handle.status == GoalStatus.STATUS_CANCELING:
                        self.get_logger().info("导航已被取消")
                        goal_handle.canceled()
                        return self.create_result(False, '导航已被取消', index, T1)
                    result = await self.navigation(path)
                    if result is None or not result.success:
                        goal_handle.abort()
                        return self.create_result(False, '导航到路径点失败', index, T1)
                    path = []

                if goal_handle.status == GoalStatus.STATUS_CANCELING:
                    self.get_logger().info("导航已被取消")
                    goal_handle.canceled()
                    return self.create_result(False, '导航已被取消', index, T1)
                result = await self.correction_client.call_async(CorrectionOdom.Request(waypoint_name=waypoint.name))
                if result is None or not result.success:
                    goal_handle.abort()
                    msg = result.message if result else '机器人矫正Odom失败!'
                    return self.create_result(False, msg, index, T1)

            if not appended:
                path.append(waypoint.pose)
            else:
                feedback = NavigationToWaypoint.Feedback()
                feedback.current_waypoint_name = waypoint.name
                feedback.pose = self.odom
                feedback.time_spent = (time.process_time() - T1) * 1000
                feedback.index = index
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(f'成功前往路径点 {waypoint.name}, 开始前往下一个路径点.')

        if path:
            if goal_handle.status == GoalStatus.STATUS_CANCELING:
                self.get_logger().info("导航已被取消")
                goal_handle.canceled()
                return self.create_result(False, '导航已被取消', len(waypoints), T1)
            result = await self.navigation(path)
            if result is None or not result.success:
                goal_handle.abort()
                return self.create_result(False, '导航到路径点失败', len(waypoints), T1)

        self.get_logger().info(f'导航到路径点 {waypoints[-1].name} 成功!')
        goal_handle.succeed()
        return self.create_result(True, '导航到路径点成功', 0, T1)

    def create_result(self, success: bool, message: str, error_code: int,
                      start_time: float) -> NavigationToWaypoint.Result:
        """
        生成返回数据
        """
        result = NavigationToWaypoint.Result()
        result.success = success
        result.message = message
        result.error_code = error_code
        result.total_time = (time.process_time() - start_time) * 1000
        result.final_pose = self.odom
        return result

    async def generate_path(self, goal_name: str) -> list[Waypoint]:
        """
        调用生成路径服务，获取从当前位置到目标路径点的路径。
        """
        start_waypoint = self.find_nearest_waypoint()
        request = GeneratePath.Request(start=start_waypoint.name, goal=goal_name)
        response = await self.generate_path_client.call_async(request)
        if not response:
            self.get_logger().error(f'生成路径服务调用失败')
            return []
        elif not response.path.waypoints:
            self.get_logger().error(f'生成路径服务返回的路径点为空')
            return []

        return response.path.waypoints

    async def navigation(self, poses: list[Pose], reverse=False):
        """
        移动机器人指定角度和距离。
        """
        goal_msg = NavPTPCMD.Goal()

        for p in poses:
            pose2d = Pose2D(x=float(p.x), y=float(p.y), theta=float(0))
            goal_msg.points.append(pose2d)

        # 这里要获取导航最后一个点的角度并赋给heading
        goal_msg.linear_vel = self.speed
        goal_msg.rotation_vel = self.speed * 5.0
        goal_msg.linear_acc = float(0.69)  # 直线加速度
        goal_msg.linear_decel = float(0.43)  # 直线减加速度
        goal_msg.rotate_acc = float(2.5)  # 旋转加速度
        goal_msg.rotate_decel = float(1)  # 旋转减加速度
        goal_msg.heading = float(poses[-1].w)
        goal_msg.back = reverse

        try:
            handle: ClientGoalHandle = await self.navigation_action.send_goal_async(goal_msg)
            self.goal_handle = handle
            return (await handle.get_result_async()).result
        except Exception as e:
            self.get_logger().error(f'发送导航目标失败: {e}')
            return None

    def find_nearest_waypoint(self) -> Waypoint:
        if not self.waypoints.waypoints:
            self.get_logger().error('路径点数据为空!')
            return Waypoint()
        print(self.odom)
        min_point: Waypoint = min(self.waypoints.waypoints,
                                  key=lambda wp: math.hypot(wp.pose.x - self.odom.x, wp.pose.y - self.odom.y))
        self.get_logger().info(f"检查当前点可能是 {min_point.name}, 距离: "
                               f"{math.hypot(min_point.pose.x - self.odom.x, min_point.pose.y - self.odom.y)}")
        return min_point


def main():
    rclpy.init()
    node = NavigationToPoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
