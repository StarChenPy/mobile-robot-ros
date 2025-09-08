import math

import rclpy
from corn_robot_toolbox.LidarToolbox import LidarToolbox
from corn_robot_toolbox.type.Direction import Direction
from corn_robot_toolbox.util import Math
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation

from chassis_msgs.srv import ResetOdom
from corn_robot_interfaces.msg import WaypointArray, CorrectedData
from corn_robot_interfaces.srv import CorrectionOdom
from web_message_transform_ros2.msg import Pose, RobotData


class CorrectionOdomNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("correction_odom")
        self.get_logger().info('里程计矫正节点 正在初始化.')

        self.lidar = LidarToolbox(self)
        self.robot_data = RobotData()
        self.waypoints = WaypointArray()

        self.declare_parameter('correction_service', '/correction_odom')
        correction_service = self.get_parameter('correction_service').value

        self.declare_parameter('robot_data_topic', '/web_transform_node/robot_data')
        robot_data_topic = self.get_parameter('robot_data_topic').value

        self.declare_parameter('set_odom_service', '/chassis/reset_odom')
        set_odom_service = self.get_parameter('set_odom_service').value

        self.declare_parameter('waypoints_topic', '/waypoints')
        waypoints_topic = self.get_parameter('waypoints_topic').value

        self.server = self.create_service(CorrectionOdom, correction_service, self.correction_callback)
        self.robot_data_sub = self.create_subscription(RobotData, robot_data_topic, self.odom_callback, 10)
        self.waypoints_sub = self.create_subscription(WaypointArray, waypoints_topic, self.waypoints_callback, 10)
        self.set_odom_client = self.create_client(ResetOdom, set_odom_service)
        self.initialpose_topic = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info('里程计矫正节点 已启动.')

    def waypoints_callback(self, msg: WaypointArray):
        self.waypoints = msg

    def odom_callback(self, msg: Pose):
        self.robot_data = msg

    def correction_callback(self, request: CorrectionOdom.Request, response: CorrectionOdom.Response):
        self.get_logger().info(f"接收到矫正请求: {request.waypoint_name}")

        x_buffer = 0
        y_buffer = 0
        angle_from_wall = 0

        if not self.waypoints.waypoints:
            self.get_logger().warning("路径点为空!")
            response.success = False
            return response

        waypoint = next((item for item in self.waypoints.waypoints if item.name == request.waypoint_name), None)

        if not waypoint:
            self.get_logger().warning("没有找到对应的矫正点，无法矫正!")
            response.success = False
            return response

        corrected_data: CorrectedData = waypoint.corrected_data
        if corrected_data.front:
            scan_angle = 10 if corrected_data.front > 0.8 else 30
            distance_from_wall = self.lidar.get_distance_from_wall(Direction.FRONT, scan_angle)
            if distance_from_wall:
                angle_from_wall = self.lidar.get_angle_from_wall(Direction.FRONT, scan_angle) - corrected_data.front_angle
                x_buffer = corrected_data.front - distance_from_wall

        if corrected_data.back and self.robot_data.sonar:
            sonar_l = self.robot_data.sonar[0]
            sonar_r = self.robot_data.sonar[1]
            self.get_logger().debug(f"扫描到的超声距离为 sonar_l: {sonar_l} sonar_r: {sonar_r}")
            distance_from_wall = Math.distance_from_origin(-5, sonar_l, 5, sonar_r) + 0.222
            x_buffer = distance_from_wall - corrected_data.back

        if corrected_data.left:
            scan_angle = 10 if corrected_data.left > 0.8 else 30
            distance_from_wall = self.lidar.get_distance_from_wall(Direction.LEFT, scan_angle)
            if distance_from_wall:
                angle_from_wall = self.lidar.get_angle_from_wall(Direction.LEFT, scan_angle) - corrected_data.left_angle
                y_buffer = corrected_data.left - distance_from_wall

        if corrected_data.right:
            scan_angle = 10 if corrected_data.right > 0.8 else 30
            distance_from_wall = self.lidar.get_distance_from_wall(Direction.RIGHT, scan_angle)
            if distance_from_wall:
                angle_from_wall = self.lidar.get_angle_from_wall(Direction.RIGHT, scan_angle) - corrected_data.right_angle
                y_buffer = distance_from_wall - corrected_data.right

        odom_data = self.robot_data.odom
        pose = waypoint.pose
        yaw = pose.w
        if angle_from_wall == 0:
            self.get_logger().warn("无法获取到墙面角度，跳过矫正角度!")
        elif abs(angle_from_wall) > 10:
            self.get_logger().warn(f"墙角度 {angle_from_wall } 与目标差距过大，不可信，跳过矫正角度! ")
        else:
            yaw = pose.w - angle_from_wall
            yaw = Math.normalize_angle(yaw)

        x, y = odom_data.x, odom_data.y
        right_angle = Math.round_right_angle(yaw)
        if right_angle == 0:
            x = pose.x + x_buffer
            y = pose.y + y_buffer
        elif right_angle == 90:
            x = pose.x - y_buffer
            y = pose.y + x_buffer
        elif right_angle == 180 or right_angle == -180:
            x = pose.x - x_buffer
            y = pose.y - y_buffer
        elif right_angle == -90:
            x = pose.x + y_buffer
            y = pose.y - x_buffer

        self.get_logger().info(f"矫正当前坐标为: x: {x}, y: {y}, w: {yaw}")
        req = ResetOdom.Request(clear_mode=0, x=x, y=y, theta=math.radians(yaw))
        self.set_odom_client.call_async(req)
        self.set_initial_pose(x, y, yaw)
        response.success = True
        return response

    def set_initial_pose(self, x, y, yaw):
        """
        设置初始位置
        @param pose 初始位置
        """
        pose_data = PoseWithCovarianceStamped()

        pose_data.header.frame_id = 'map'
        pose_data.header.stamp = self.get_clock().now().to_msg()
        pose_data.pose.pose.position.x = x
        pose_data.pose.pose.position.y = y
        pose_data.pose.pose.position.z = 0.0

        # 将航向角转换为四元数
        quat = Rotation.from_euler('z', yaw, degrees=True).as_quat()
        pose_data.pose.pose.orientation.x = quat[0]
        pose_data.pose.pose.orientation.y = quat[1]
        pose_data.pose.pose.orientation.z = quat[2]
        pose_data.pose.pose.orientation.w = quat[3]

        # 协方差（根据需要可以调大或调小）
        # 只需关心第 0 位（x 方向方差）第 7 位（y 方向方差）第 35 位（yaw 方向方差）
        pose_data.pose.covariance = [0.0] * 36
        pose_data.pose.covariance[0] = 0.002  # x 方向，标准差 5cm
        pose_data.pose.covariance[7] = 0.002  # y 方向，标准差 2cm
        pose_data.pose.covariance[35] = math.radians(2) ** 2  # 偏航角，标准差 2°

        self.initialpose_topic.publish(pose_data)

def main():
    rclpy.init()
    node = CorrectionOdomNode()

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
