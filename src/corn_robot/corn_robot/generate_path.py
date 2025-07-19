import copy

import rclpy.node

from collections import deque

from rcl_interfaces.msg import ParameterDescriptor

from corn_robot_interfaces.srv import GeneratePath
from corn_robot_interfaces.msg import WaypointArray


class GeneratePathNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('generate_path')
        self.get_logger().info('路径生成节点 已启动.')
        self.waypoints = WaypointArray()

        self.declare_parameter('waypoints_topic', 'waypoints',
                               ParameterDescriptor(description='发布路径点的话题名.'))
        self.declare_parameter('generate_path_service', 'generate_path',
                               ParameterDescriptor(description='生成路径到的服务名.'))

        waypoints_topic = self.get_parameter("waypoints_topic").value
        generate_path_service = self.get_parameter("generate_path_service").value

        self.sub = self.create_subscription(WaypointArray, waypoints_topic, self.sub_waypoint_callback, 10)
        self.server = self.create_service(GeneratePath, generate_path_service, self.generate_path_callback)

    def sub_waypoint_callback(self, msg):
        self.waypoints = msg

    def generate_path_callback(self, request: GeneratePath.Request, response: GeneratePath.Response):
        if not request.start:
            self.get_logger().warn('起点为空，无法生成路径!')
            return response
        if not request.goal:
            self.get_logger().warn('终点为空，无法生成路径!')
            return response

        path = self.bfs(request.start, request.goal)
        if path:
            path_names = [i.name for i in path.waypoints]
            self.get_logger().info(f'从 {request.start} 到 {request.goal} 的路径是: {" -> ".join(path_names)}')
        else:
            self.get_logger().warn(f'没找到从 {request.start} 到 {request.goal} 的路径。')
        response.path = path
        return response

    def bfs(self, start_name: str, goal_name: str) -> WaypointArray:
        """
        使用广度优先搜索算法查找从 start_name 到 goal_name 的路径。

        功能扩展：
        - 如果 connected_nodes 中节点名称以 '-' 开头，表示该段需要倒车，返回路径中的对应 Waypoint.is_reverse = True。
        """
        waypoint_array = WaypointArray()

        # 构建路径点名称 -> Waypoint对象 的映射，方便快速访问
        name_to_waypoint = {wp.name: wp for wp in self.waypoints.waypoints}

        # 检查起点与终点是否存在
        if start_name not in name_to_waypoint:
            self.get_logger().error(f"起点 {start_name} 不在路径点中!")
            return waypoint_array
        elif goal_name not in name_to_waypoint:
            self.get_logger().error(f"终点 {goal_name} 不在路径点中!")
            return waypoint_array

        # 队列结构：(当前节点名称, 当前路径 [(节点名称, 是否倒车)] )
        queue = deque()
        queue.append((start_name, [(start_name, False)]))  # 起点默认不倒车

        visited = set()  # 记录已访问节点，防止回环

        while queue:
            # 弹出队列，处理当前节点
            current_name, path = queue.popleft()

            # BFS标准写法：出队列时标记已访问
            if current_name in visited:
                continue
            visited.add(current_name)

            # 如果找到目标节点，构建并返回 WaypointArray
            if current_name == goal_name:
                waypoint_array.waypoints = []
                for node_name, is_reverse in path:
                    if node_name not in name_to_waypoint:
                        self.get_logger().warn(f'路径点 "{node_name}" 不存在，跳过.')
                        continue
                    # 使用 deepcopy 防止污染原始 Waypoint
                    wp = copy.deepcopy(name_to_waypoint[node_name])
                    wp.is_reverse = is_reverse  # 根据路径设置倒车标志
                    waypoint_array.waypoints.append(wp)
                return waypoint_array

            # 遍历相邻节点（connected_nodes）
            for neighbor in name_to_waypoint[current_name].connected_nodes:
                is_reverse = False
                target_name = neighbor

                # 检查是否带 '-'，如果是，表示该段需要倒车
                if neighbor.startswith('-'):
                    is_reverse = True
                    target_name = neighbor[1:]  # 去除 '-' 得到真实路径点名称

                # 检查邻接节点是否在已知路径点中
                if target_name not in name_to_waypoint:
                    self.get_logger().warn(f'路径点 "{target_name}" 不存在，请检查 "{current_name}" 的 connected_nodes 配置.')
                    continue

                # 邻接节点未访问，加入队列继续搜索
                if target_name not in visited:
                    queue.append((target_name, path + [(target_name, is_reverse)]))

        self.get_logger().warn(f"未找到从 {start_name} 到 {goal_name} 的路径.")
        return waypoint_array


def main():
    rclpy.init()

    node = GeneratePathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
