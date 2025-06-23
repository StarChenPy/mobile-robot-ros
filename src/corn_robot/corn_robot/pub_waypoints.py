import math
import os

import yaml
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from corn_robot_interfaces.msg import Waypoint, WaypointArray
from tf_transformations import quaternion_from_euler

import rclpy.node


def resolve_waypoints_path(raw_path: str) -> str:
    if os.path.isabs(raw_path):
        return raw_path
    return os.path.join(get_package_share_directory("corn_robot"), "param", raw_path)


class PubWaypointsNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('pub_waypoints_node')
        self.get_logger().info('导航点发布节点 已启动.')
        self.raw_waypoints = {}

        self.declare_parameter('waypoints_path', "行业赛路径点.yaml",
                               ParameterDescriptor(description='路径点yaml文件的路径.'))
        self.declare_parameter('waypoints_topic', 'waypoints',
                               ParameterDescriptor(description='发布路径点的话题名.'))

        # 创建发布者
        waypoints_topic = self.get_parameter("waypoints_topic").value
        self.publisher = self.create_publisher(WaypointArray, waypoints_topic, 10)

        # 定时器，定期发布消息
        self.read_yaml()
        self.publish_waypoints()
        self.timer = self.create_timer(10, self.publish_waypoints)

    def read_yaml(self):
        try:
            raw_path = self.get_parameter('waypoints_path').value
            waypoints_path = resolve_waypoints_path(raw_path)
            with open(waypoints_path, 'r', encoding='utf-8') as f:
                self.raw_waypoints = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"读取路径点 YAML 文件失败: {e}")
            self.raw_waypoints = {}

    def publish_waypoints(self):
        waypoint_array = WaypointArray()

        if not self.raw_waypoints:
            self.get_logger().error("路径点数据为空!")

        for k, v in self.raw_waypoints.items():
            msg = Waypoint()

            msg.name = k
            pose_data = v.get('pose')
            x = pose_data.get('x')
            y = pose_data.get('y')
            yaw_deg = pose_data.get('yaw')

            msg.pose.position = Point(x=x, y=y, z=0.0)
            quat = quaternion_from_euler(0, 0, math.radians(yaw_deg))
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]

            msg.connected_nodes = v.get('connected_nodes', [])

            waypoint_array.waypoints.append(msg)

            self.get_logger().debug(f"已从参数加载路径点: {msg.name} (x={x}, y={y}, yaw={yaw_deg}°)")

        self.publisher.publish(waypoint_array)


def main():
    rclpy.init()

    node = PubWaypointsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
