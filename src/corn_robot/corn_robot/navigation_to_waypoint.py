import math
import time
from typing import List, Optional

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from corn_robot_interfaces.action import NavigationToWaypoint, RobotMotion
from corn_robot_interfaces.msg import WaypointArray, Waypoint
from corn_robot_interfaces.srv import GeneratePath


def normalize_angle(angle: float) -> float:
    """将角度标准化到 [-π, π]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def create_result(success: bool, message: str, error_code: int, start_time: float, pose=None) -> NavigationToWaypoint.Result:
    result = NavigationToWaypoint.Result()
    result.success = success
    result.message = message
    result.error_code = error_code
    result.total_time = (time.process_time() - start_time) * 1000
    result.final_pose = pose if pose else PoseStamped()
    return result


def compute_final_orientation(pose_stamped: PoseStamped, waypoint: Waypoint) -> float:
    _, _, current_yaw = euler_from_quaternion([
        pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w
    ])
    _, _, target_yaw = euler_from_quaternion([
        waypoint.pose.orientation.x, waypoint.pose.orientation.y,
        waypoint.pose.orientation.z, waypoint.pose.orientation.w
    ])
    return math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))


def compute_distance(pose_stamped: PoseStamped, waypoint: Waypoint) -> float:
    x0, y0 = pose_stamped.pose.position.x, pose_stamped.pose.position.y
    x1, y1 = waypoint.pose.position.x, waypoint.pose.position.y
    return math.hypot(x1 - x0, y1 - y0)


def compute_delta_theta(pose_stamped: PoseStamped, waypoint: Waypoint) -> float:
    orientation_q = pose_stamped.pose.orientation
    _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    x0, y0 = pose_stamped.pose.position.x, pose_stamped.pose.position.y
    x1, y1 = waypoint.pose.position.x, waypoint.pose.position.y
    target_angle = math.atan2(y1 - y0, x1 - x0)
    return math.atan2(math.sin(target_angle - current_yaw), math.cos(target_angle - current_yaw))


class NavigationToPoseNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('navigation_to_waypoint')
        self.get_logger().info('路径点导航节点 已启动.')

        self.goal_handle = None
        self.waypoints = WaypointArray()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 参数
        self.declare_parameter('navigation_to_waypoint_action', 'navigation_to_waypoint')
        self.declare_parameter('generate_path_service', 'generate_path')
        self.declare_parameter('waypoints_topic', 'waypoints')
        self.declare_parameter('robot_motion_action', 'robot_motion')
        self.declare_parameter('speed', 0.4)
        self.declare_parameter('distance_threshold', 0.05)

        nav2wp_action = self.get_parameter('navigation_to_waypoint_action').value
        generate_path_service = self.get_parameter('generate_path_service').value
        robot_motion_action = self.get_parameter('robot_motion_action').value
        waypoints_topic = self.get_parameter('waypoints_topic').value

        # 缓存参数
        self.speed = self.get_parameter('speed').value
        self.distance_threshold = self.get_parameter('distance_threshold').value

        # 通信接口
        self.action_server = ActionServer(self, NavigationToWaypoint, nav2wp_action,
                                          execute_callback=self.execute_callback,
                                          cancel_callback=self.cancel_callback)
        self.generate_path_client = self.create_client(GeneratePath, generate_path_service)
        self.motion_action_client = ActionClient(self, RobotMotion, robot_motion_action)
        self.waypoints_sub = self.create_subscription(WaypointArray, waypoints_topic, self.waypoints_callback, 10)

    def waypoints_callback(self, msg: WaypointArray):
        self.waypoints = msg

    async def cancel_callback(self, _):
        self.get_logger().info('接收到取消导航到路径点请求.')
        if self.goal_handle is not None:
            await self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        return rclpy.action.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> NavigationToWaypoint.Result:
        goal: NavigationToWaypoint.Goal = goal_handle.request
        self.get_logger().info(f'接收到导航到路径点请求: {goal.waypoint_name}')
        T1 = time.process_time()

        self.speed = goal.speed if goal.speed > 0 else self.get_parameter('speed').value
        pose_stamped = self.get_pose_stamped()
        if pose_stamped is None:
            goal_handle.abort()
            return create_result(False, '获取当前位置失败!', -1, T1)

        try:
            waypoints = await self.generate_path(pose_stamped.pose.position.x, pose_stamped.pose.position.y, goal.waypoint_name)
        except RuntimeError as e:
            goal_handle.abort()
            return create_result(False, str(e), -1, T1, pose_stamped)

        for index, waypoint in enumerate(waypoints):
            self.get_logger().info(f'准备前往路径点 {waypoint.name} (倒车: {waypoint.is_reverse})')
            pose_stamped = self.get_pose_stamped()
            if pose_stamped is None:
                goal_handle.abort()
                return create_result(False, '获取当前位置失败!', index, T1)

            distance = compute_distance(pose_stamped, waypoint)
            if distance < self.distance_threshold:
                self.get_logger().info(f'已在路径点 {waypoint.name} 附近, 跳过.')
                continue

            # 计算应转动角度
            delta_theta = compute_delta_theta(pose_stamped, waypoint)

            # 倒车逻辑
            if waypoint.is_reverse:
                # 与前进方向相反的角度
                delta_theta = normalize_angle(delta_theta - math.pi)
                # 通过传递负距离实现倒车
                distance = -distance

            # 执行移动（正数为前进，负数为后退）
            result = await self.move_robot(math.degrees(delta_theta), distance)

            if result is None or not result.success:
                goal_handle.abort()
                msg = result.message if result else '机器人运动服务调用失败!'
                return create_result(False, msg, index, T1, pose_stamped)

            # 反馈信息
            feedback = NavigationToWaypoint.Feedback()
            feedback.current_waypoint_name = waypoint.name
            feedback.pose = pose_stamped
            feedback.time_spent = (time.process_time() - T1) * 1000
            feedback.index = index
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'成功前往路径点 {waypoint.name}, 开始前往下一个路径点.')

        pose_stamped = self.get_pose_stamped()
        if pose_stamped is None:
            goal_handle.abort()
            return create_result(False, '获取当前位置失败!', len(waypoints), T1)
        distance = compute_distance(pose_stamped, waypoints[-1])
        # 计算应转动角度
        delta_theta = compute_delta_theta(pose_stamped, waypoints[-1])

        result = await self.move_robot(math.degrees(delta_theta), distance)

        if result is None or not result.success:
            goal_handle.abort()
            msg = result.message if result else '机器人运动服务调用失败!'
            return create_result(False, msg, len(waypoints), T1, pose_stamped)

        # 最后调整终点位姿
        final_pose = self.get_pose_stamped()
        if final_pose is None:
            goal_handle.abort()
            return create_result(False, '获取当前位置失败!', len(waypoints), T1)

        delta_theta = compute_final_orientation(final_pose, waypoints[-1])
        result = await self.move_robot(math.degrees(delta_theta), 0.0)

        if result is None or not result.success:
            goal_handle.abort()
            msg = result.message if result else '机器人运动服务调用失败!(终点调整朝向)'
            return create_result(False, msg, -1, T1, final_pose)

        self.get_logger().info(f'导航到路径点 {waypoints[-1].name} 成功!')
        goal_handle.succeed()
        return create_result(True, '导航到路径点成功', 0, T1, final_pose)

    async def generate_path(self, x: float, y: float, goal_name: str) -> List[Waypoint]:
        start_waypoint = self.find_nearest_waypoint(x, y)
        request = GeneratePath.Request(start=start_waypoint.name, goal=goal_name)
        response = await self.generate_path_client.call_async(request)
        if not response or not response.path.waypoints:
            raise RuntimeError('路径生成失败或为空')
        return response.path.waypoints

    def get_pose_stamped(self) -> Optional[PoseStamped]:
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().error(f'获取 TF 失败: {e}')
            return None

    async def move_robot(self, angle: float, distance: float) -> Optional[RobotMotion.Result]:
        goal = RobotMotion.Goal()
        goal.angle = angle
        goal.distance = distance
        goal.speed = self.speed
        try:
            handle: ClientGoalHandle = await self.motion_action_client.send_goal_async(goal)
            self.goal_handle = handle
            return (await handle.get_result_async()).result
        except Exception as e:
            self.get_logger().error(f'发送运动目标失败: {e}')
            return None

    def find_nearest_waypoint(self, x: float, y: float) -> Waypoint:
        if not self.waypoints.waypoints:
            self.get_logger().error('路径点数据为空!')
            return Waypoint()
        return min(self.waypoints.waypoints, key=lambda wp: math.hypot(wp.pose.position.x - x, wp.pose.position.y - y))


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
