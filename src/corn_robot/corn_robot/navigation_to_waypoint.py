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

from base_nav2.action import NavCMD
from base_nav2_ptp.action import NavPTPCMD
from chassis_msgs.srv import ResetOdom
from corn_robot_interfaces.action import NavigationToWaypoint
from corn_robot_interfaces.msg import Waypoint
from corn_robot_interfaces.srv import GeneratePath, CorrectionOdom, FindNearestWaypoint
from corn_robot_toolbox.util import Math
from web_message_transform_ros2.msg import Pose, RobotData


def is_correcter_point(waypoint: Waypoint) -> bool:
    data = waypoint.corrected_data
    return data.front or data.back or data.left or data.right


class NavigationToPoseNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('navigation_to_waypoint')
        self.get_logger().info('路径点导航节点 正在初始化.')

        self.goal_handle = None
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

        self.declare_parameter('find_nearest_waypoint_service', '/find_nearest_waypoint')
        find_nearest_waypoint_service = self.get_parameter('find_nearest_waypoint_service').value

        self.declare_parameter('navigation_action', '/nav2_action')
        navigation_action = self.get_parameter('navigation_action').value

        self.declare_parameter('navigation_ptp_action', '/nav2_ptp_action')
        navigation_ptp_action = self.get_parameter('navigation_ptp_action').value

        self.declare_parameter("reverse_uphill", False)

        self.declare_parameter('speed', 0.4)
        self.declare_parameter('uphill_offset', 0.1)
        self.declare_parameter("distance_threshold", 0.05)
        self.declare_parameter("skip_start_point_correction", "")

        # 缓存参数
        self.speed = self.get_parameter('speed').value

        # 通信接口
        self.action_server = ActionServer(self, NavigationToWaypoint, nav2wp_action,
                                          execute_callback=self.execute_callback,
                                          cancel_callback=self.cancel_callback)
        self.navigation_action_client = ActionClient(self, NavCMD, navigation_action)
        self.navigation_ptp_action_client = ActionClient(self, NavPTPCMD, navigation_ptp_action)
        self.generate_path_client = self.create_client(GeneratePath, generate_path_service)
        self.correction_client = self.create_client(CorrectionOdom, correction_service)
        self.find_nearest_waypoint_client = self.create_client(FindNearestWaypoint, find_nearest_waypoint_service)
        self.set_odom_client = self.create_client(ResetOdom, set_odom_service)
        self.robot_data_sub = self.create_subscription(RobotData, robot_data_topic, self.robot_data_callback, 10)

        self.get_logger().info('路径点导航节点 已启动.')

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
        self.get_logger().info(f'接收到导航到路径点请求: {goal.goal_name}')
        T1 = time.process_time()

        self.speed = goal.speed if goal.speed > 0 else self.get_parameter('speed').value
        waypoints = await self.generate_path(goal.start_name, goal.goal_name)
        if not waypoints:
            return self.create_result(False, '生成路径失败', -1, T1)

        path = []
        in_slope = False
        for index, waypoint in enumerate(waypoints):
            if len(waypoints) != 1 and index == 0:
                if waypoint.on_slope:
                    in_slope = True
                continue
            elif index == 0 and self.get_parameter("distance_threshold").value > 0:
                distance = math.hypot(waypoint.pose.x - self.odom.x, waypoint.pose.y - self.odom.y)
                if distance < self.get_parameter("distance_threshold").value:
                    continue

            if goal_handle.status == GoalStatus.STATUS_CANCELING:
                self.get_logger().info("导航已被取消")
                goal_handle.canceled()
                return self.create_result(False, '导航已被取消', index, T1)

            appended = False

            # 判断是否为坡道点
            if waypoint.on_slope:
                if not in_slope:
                    self.get_logger().info(f"上坡，额外行走 {self.get_parameter('uphill_offset').value} m")
                    in_slope = True

                    if waypoint.pose.w != 0:
                        w_back = waypoint.pose.w
                    elif index < len(waypoints) - 1:
                        w_back = Math.compute_delta_theta(waypoint.pose, waypoints[index + 1].pose)
                    else:
                        w_back = 0.0
                    waypoint.pose.w = Math.compute_delta_theta(waypoints[index - 1].pose, waypoint.pose)
                    offset_pose = Math.get_target_coordinate(waypoint.pose, self.get_parameter('uphill_offset').value)
                    offset_pose.w = w_back
                    waypoint.pose.w = w_back
                    if not appended:
                        appended = True
                        path.append(offset_pose)
                        if goal_handle.status == GoalStatus.STATUS_CANCELING:
                            self.get_logger().info("导航已被取消")
                            goal_handle.canceled()
                            return self.create_result(False, '导航已被取消', index, T1)
                        result = await self.auto_navigation(path, self.get_parameter("reverse_uphill").value)
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
            if index >= 2:
                pass
            elif len(waypoints) > 1:
                if Math.is_behind(waypoints[index - 1].pose, waypoint.pose, 80):
                    self.get_logger().info(f"触发了倒车，当前pose: {waypoints[index - 1].pose}, 目标 pose: {waypoint.pose}")
                    if not appended:
                        appended = True
                        path.append(waypoint.pose)
                        if goal_handle.status == GoalStatus.STATUS_CANCELING:
                            self.get_logger().info("导航已被取消")
                            goal_handle.canceled()
                            return self.create_result(False, '导航已被取消', index, T1)
                        # 如果不是最后一个点，并且不是矫正点，则朝向下一个点的角度
                        if index < len(waypoints) - 1 and not is_correcter_point(waypoint):
                            waypoint.pose.w = Math.compute_delta_theta(waypoint.pose, waypoints[index + 1].pose)
                        result = await self.auto_navigation(path, True)
                        if result is None or not result.success:
                            goal_handle.abort()
                            return self.create_result(False, '导航到路径点失败', index, T1)
                        path = []
            else:
                if Math.is_behind(self.odom, waypoint.pose, 80):
                    self.get_logger().info(f"触发了倒车，当前pose: {self.odom}, 目标 pose: {waypoint.pose}")
                    if not appended:
                        appended = True
                        path.append(waypoint.pose)
                        if goal_handle.status == GoalStatus.STATUS_CANCELING:
                            self.get_logger().info("导航已被取消")
                            goal_handle.canceled()
                            return self.create_result(False, '导航已被取消', index, T1)
                        result = await self.auto_navigation(path, True)
                        if result is None or not result.success:
                            goal_handle.abort()
                            return self.create_result(False, '导航到路径点失败', index, T1)
                        path = []

            # 是否为矫正点
            if is_correcter_point(waypoint):
                if index == len(waypoints) and waypoint.name == self.get_parameter('skip_start_point_correction').value:
                    self.get_logger().info("最后一个矫正点为起始点，跳过矫正.")
                    path.append(waypoint.pose)
                    continue

                if not appended:
                    appended = True
                    path.append(waypoint.pose)
                    if goal_handle.status == GoalStatus.STATUS_CANCELING:
                        self.get_logger().info("导航已被取消")
                        goal_handle.canceled()
                        return self.create_result(False, '导航已被取消', index, T1)
                    result = await self.auto_navigation(path)
                    if result is None or not result.success:
                        goal_handle.abort()
                        return self.create_result(False, '导航到路径点失败', index, T1)
                    path = []

                if goal_handle.status == GoalStatus.STATUS_CANCELING:
                    self.get_logger().info("导航已被取消")
                    goal_handle.canceled()
                    return self.create_result(False, '导航已被取消', index, T1)
                time.sleep(0.3)
                result = await self.correction_client.call_async(CorrectionOdom.Request(waypoint_name=waypoint.name))
                if result is None or not result.success:
                    goal_handle.abort()
                    msg = result.message if result else '机器人矫正Odom失败!'
                    return self.create_result(False, msg, index, T1)
                time.sleep(0.3)

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
            result = await self.auto_navigation(path)
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

    async def generate_path(self, start_name: str, goal_name: str) -> list[Waypoint]:
        """
        调用生成路径服务，获取从当前位置到目标路径点的路径。
        """
        # 寻找当前点位
        if not start_name:
            waypoint_request = await self.find_nearest_waypoint_client.call_async(FindNearestWaypoint.Request())
            start_name = waypoint_request.waypoint.name
        # 寻找路径
        request = GeneratePath.Request(start=start_name, goal=goal_name)
        response = await self.generate_path_client.call_async(request)
        if not response:
            self.get_logger().error(f'生成路径服务调用失败')
            return []
        elif not response.path.waypoints:
            self.get_logger().error(f'生成路径服务返回的路径点为空')
            return []

        return response.path.waypoints

    def auto_navigation(self, poses: list[Pose], reverse=False):
        if len(poses) > 1:
            return self.path_following_navigation(poses, reverse)
        else:
            return self.ptp_navigation(poses, reverse)

    async def path_following_navigation(self, poses: list[Pose], reverse=False):
        """
        移动机器人路径跟随导航，实时规划。
        """
        goal_msg = NavCMD.Goal()

        for p in poses:
            pose2d = Pose2D(x=float(p.x), y=float(p.y), theta=float(0))
            goal_msg.points.append(pose2d)

        goal_msg.linear_vel = self.speed - 0.05
        goal_msg.rotation_vel = self.speed * 5.0
        goal_msg.rotate_acc = float(2.5)  # 旋转加速度
        goal_msg.rotate_decel = float(2.0)  # 旋转减加速度
        # 这里要获取导航最后一个点的角度并赋给heading
        goal_msg.heading = float(poses[-1].w)
        goal_msg.back = reverse

        try:
            handle: ClientGoalHandle = await self.navigation_action_client.send_goal_async(goal_msg)
            self.goal_handle = handle
            return (await handle.get_result_async()).result
        except Exception as e:
            self.get_logger().error(f'发送导航目标失败: {e}')
            return None

    async def ptp_navigation(self, poses: list[Pose], reverse=False):
        """
        移动机器人点对点导航，预规划。
        """
        goal_msg = NavPTPCMD.Goal()

        for p in poses:
            pose2d = Pose2D(x=float(p.x), y=float(p.y), theta=float(0))
            goal_msg.points.append(pose2d)

        goal_msg.linear_vel = self.speed
        goal_msg.rotation_vel = self.speed * 5.0
        goal_msg.linear_acc = float(3)  # 直线加速度
        goal_msg.linear_decel = float(2)  # 直线减加速度
        goal_msg.rotate_acc = float(2.5)  # 旋转加速度
        goal_msg.rotate_decel = float(2.0)  # 旋转减加速度
        # 这里要获取导航最后一个点的角度并赋给heading
        goal_msg.heading = float(poses[-1].w)
        goal_msg.back = reverse

        try:
            handle: ClientGoalHandle = await self.navigation_ptp_action_client.send_goal_async(goal_msg)
            self.goal_handle = handle
            return (await handle.get_result_async()).result
        except Exception as e:
            self.get_logger().error(f'发送导航目标失败: {e}')
            return None


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
