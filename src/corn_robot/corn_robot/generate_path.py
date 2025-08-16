from typing import Dict, List

import rclpy.node
from astar import AStar
from rcl_interfaces.msg import ParameterDescriptor

from corn_robot_interfaces.msg import WaypointArray, Waypoint
from corn_robot_interfaces.srv import GeneratePath


class WaypointWrapper:
    def __init__(self, msg: Waypoint):
        self.msg = msg

    def __eq__(self, other):
        if not isinstance(other, WaypointWrapper):
            return False
        return self.msg.name == other.msg.name

    def __hash__(self):
        return hash(self.msg.name)

    @property
    def name(self):
        return self.msg.name

    @property
    def pose(self):
        return self.msg.pose

    @property
    def connected_nodes(self):
        return self.msg.connected_nodes

    @property
    def on_slope(self):
        return self.msg.on_slope


def manhattan(wp1: WaypointWrapper, wp2: WaypointWrapper) -> float:
    x1, y1 = wp1.pose.x, wp1.pose.y
    x2, y2 = wp2.pose.x, wp2.pose.y
    return abs(x1 - x2) + abs(y1 - y2)


class WaypointAStar(AStar[WaypointWrapper]):
    def __init__(self, waypoints: List[Waypoint]) -> None:
        self.waypoints_list = [WaypointWrapper(wp) for wp in waypoints]
        self.name_to_wp: Dict[str, WaypointWrapper] = {wp.name: wp for wp in self.waypoints_list}

    def find_path_by_name(self, start_name: str, goal_name: str):
        s = self.name_to_wp.get(start_name)
        g = self.name_to_wp.get(goal_name)
        if s is None or g is None:
            return None
        return self.astar(s, g)

    def heuristic_cost_estimate(self, current: WaypointWrapper, goal: WaypointWrapper) -> float:
        return manhattan(current, goal)

    def distance_between(self, n1: WaypointWrapper, n2: WaypointWrapper) -> float:
        cost = manhattan(n1, n2)
        if n2.on_slope:
            cost += 3
        return cost

    def neighbors(self, node: WaypointWrapper) -> List[WaypointWrapper]:
        # connected_nodes 是名字，需要从字典查对象
        return [self.name_to_wp[name] for name in node.connected_nodes if name in self.name_to_wp]

    def is_goal_reached(self, current: WaypointWrapper, goal: WaypointWrapper) -> bool:
        return current.name == goal.name


class GeneratePathNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('generate_path')
        self.get_logger().info('路径生成节点 正在初始化.')
        self.waypoints = WaypointArray()

        self.declare_parameter('waypoints_topic', 'waypoints',
                               ParameterDescriptor(description='发布路径点的话题名.'))
        self.declare_parameter('generate_path_service', 'generate_path',
                               ParameterDescriptor(description='生成路径到的服务名.'))

        waypoints_topic = self.get_parameter("waypoints_topic").value
        generate_path_service = self.get_parameter("generate_path_service").value

        self.sub = self.create_subscription(WaypointArray, waypoints_topic, self.sub_waypoint_callback, 10)
        self.server = self.create_service(GeneratePath, generate_path_service, self.generate_path_callback)

        self.get_logger().info('路径生成节点 已启动.')

    def sub_waypoint_callback(self, msg):
        self.waypoints = msg

    def generate_path_callback(self, request: GeneratePath.Request, response: GeneratePath.Response):
        if not request.start:
            self.get_logger().warn('起点为空，无法生成路径!')
            return response
        if not request.goal:
            self.get_logger().warn('终点为空，无法生成路径!')
            return response

        a_star = WaypointAStar(self.waypoints.waypoints)
        path = a_star.find_path_by_name(request.start, request.goal)
        if path:
            response.path.waypoints = [wp.msg for wp in path]
            path_names = [i.name for i in response.path.waypoints]
            self.get_logger().info(f'从 {request.start} 到 {request.goal} 的路径是: {" -> ".join(path_names)}')
        else:
            self.get_logger().warn(f'没找到从 {request.start} 到 {request.goal} 的路径。')
        return response


def main():
    rclpy.init()

    node = GeneratePathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
