import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from chassis_msgs.srv import ResetOdom
from corn_robot_interfaces.msg import WaypointArray
from corn_robot_interfaces.srv import CorrectionOdom
from corn_robot_toolbox.LidarToolbox import LidarToolbox
from corn_robot_toolbox.type.Direction import Direction
from corn_robot_toolbox.util import Math
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

        self.get_logger().info('里程计矫正节点 已启动.')

    def waypoints_callback(self, msg: WaypointArray):
        self.waypoints = msg

    def odom_callback(self, msg: Pose):
        self.robot_data = msg

    def correction_callback(self, request: CorrectionOdom.Request, response: CorrectionOdom.Response):
        self.get_logger().info(f"接收到矫正请求")

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

        corrected_data = waypoint.corrected_data
        if corrected_data.front:
            distance_from_wall = self.lidar.get_distance_from_wall(Direction.FRONT)
            if distance_from_wall:
                angle_from_wall = self.lidar.get_angle_from_wall_once(Direction.FRONT)
                x_buffer = distance_from_wall - corrected_data.front

        if corrected_data.back:
            for i in range(10):
                sonar_1 = self.robot_data.sonar
                time.sleep(0.2)
                sonar_2 = self.robot_data.sonar

                if not sonar_2 and not sonar_1:
                    continue

                if abs(sonar_1[0] - sonar_2[0]) < 0.02 and abs(sonar_1[1] - sonar_2[1]) < 0.02:
                    sonar_l = (sonar_1[0] + sonar_2[0]) / 2
                    sonar_r = (sonar_1[1] + sonar_2[1]) / 2
                    self.get_logger().debug(f"扫描到的超声距离为 sonar_l: {sonar_l} sonar_r: {sonar_r}")
                    distance_from_wall = Math.distance_from_origin(-5, sonar_l, 5, sonar_r) + 0.222
                    x_buffer = distance_from_wall - corrected_data.back
                    break
                else:
                    self.get_logger().warn(f"超声两次距离获取误差较大 sonar_l: {sonar_1[0] - sonar_2[0]} "
                                           f"sonar_r: {abs(sonar_1[1] - sonar_2[1])}，重试 {i + 1} 次")

        if corrected_data.left:
            distance_from_wall = self.lidar.get_distance_from_wall(Direction.LEFT)
            if distance_from_wall:
                angle_from_wall = self.lidar.get_angle_from_wall_once(Direction.LEFT)
                y_buffer = corrected_data.left - distance_from_wall

        if corrected_data.right:
            distance_from_wall = self.lidar.get_distance_from_wall(Direction.RIGHT)
            if distance_from_wall:
                angle_from_wall = self.lidar.get_angle_from_wall_once(Direction.RIGHT)
                y_buffer = distance_from_wall - corrected_data.right

        odom_data = self.robot_data.odom
        pose = waypoint.pose
        yaw = pose.w
        if angle_from_wall != 0:
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
            x = pose.x + x_buffer
            y = pose.y - y_buffer
        elif right_angle == -90:
            x = pose.x + y_buffer
            y = pose.y - x_buffer

        self.get_logger().info(f"矫正当前坐标为: x: {x}, y: {y}, w: {yaw}")
        req = ResetOdom.Request(clear_mode=0, x=x, y=y, theta=math.radians(yaw))
        self.set_odom_client.call_async(req)
        response.success = True
        return response


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
