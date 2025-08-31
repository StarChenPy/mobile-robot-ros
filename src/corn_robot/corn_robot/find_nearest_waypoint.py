import math

import rclpy.node

from corn_robot_interfaces.msg import WaypointArray
from corn_robot_toolbox.util import Math

from corn_robot_interfaces.srv import FindNearestWaypoint
from web_message_transform_ros2.msg import Pose, RobotData


class FindNearestWaypointNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("find_nearest_waypoint")
        self.get_logger().info('寻找当前路径点节点 正在初始化.')

        self.odom = Pose()
        self.waypoints = []

        self.declare_parameter('find_nearest_waypoint_topic', '/find_nearest_waypoint')
        find_nearest_waypoint_service = self.get_parameter('find_nearest_waypoint_topic').value

        self.declare_parameter('robot_data_topic', '/web_transform_node/robot_data')
        robot_data_topic = self.get_parameter('robot_data_topic').value

        self.declare_parameter('waypoints_topic', '/waypoints')
        waypoints_topic = self.get_parameter('waypoints_topic').value

        self.declare_parameter('angle_threshold', 30)

        self.service_server = self.create_service(FindNearestWaypoint, find_nearest_waypoint_service, self.service_callback)
        self.robot_data_sub = self.create_subscription(RobotData, robot_data_topic, self.robot_data_callback, 10)
        self.waypoints_sub = self.create_subscription(WaypointArray, waypoints_topic, self.waypoints_callback, 10)

        self.get_logger().info('寻找当前路径点节点 已启动.')

    def robot_data_callback(self, msg: RobotData):
        self.odom = msg.odom

    def waypoints_callback(self, msg: WaypointArray):
        self.waypoints = msg.waypoints

    def service_callback(self, _: FindNearestWaypoint.Request,
                         response: FindNearestWaypoint.Response) -> FindNearestWaypoint.Response:
        if not self.waypoints:
            self.get_logger().error("路径点数据为空!")
            return response

        # 候选点生成器：朝向相近的点
        candidates = (
            wp for wp in self.waypoints
            if abs(Math.normalize_angle(wp.pose.w - self.odom.w)) < self.get_parameter('angle_threshold').value
        )

        try:
            # 找出距离最近的点
            min_point = min(
                candidates,
                key=lambda wp: math.hypot(wp.pose.x - self.odom.x, wp.pose.y - self.odom.y)
            )
        except ValueError:  # candidates 为空
            self.get_logger().warning("未找到符合朝向的点，退回到全局最近点")
            min_point = min(
                self.waypoints,
                key=lambda wp: math.hypot(wp.pose.x - self.odom.x, wp.pose.y - self.odom.y)
            )

        # 计算距离只做一次
        dx, dy = min_point.pose.x - self.odom.x, min_point.pose.y - self.odom.y
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f"检查当前点 ({self.odom.x:.2f}, {self.odom.y:.2f}, {self.odom.w}) "
            f"可能是 {min_point.name} ({min_point.pose.x:.2f}, {min_point.pose.y:.2f}, {min_point.pose.w}), 距离: {dist:.2f}"
        )
        response.waypoint = min_point
        return response

def main():
    rclpy.init()

    node = FindNearestWaypointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()