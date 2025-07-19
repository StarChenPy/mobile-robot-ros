import os
from std_srvs.srv import Empty

import rclpy.node
import yaml
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor

from corn_robot_interfaces.msg import MotionArray, Motion, KeyBool, ConnectedNode


def resolve_motion_path(raw_path: str) -> str:
    if os.path.isabs(raw_path):
        return raw_path
    return os.path.join(get_package_share_directory("corn_robot_oms"), "param", raw_path)


class PubMotionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("pub_motions")
        self.get_logger().info('OMS 动作发布节点 已启动.')
        self.raw_motions = {}

        self.declare_parameter('motion_path', "国赛OMS动作.yaml",
                               ParameterDescriptor(description='OMS 动作 yaml 文件的路径.'))
        self.declare_parameter('motions_topic', 'motions',
                               ParameterDescriptor(description='发布 OMS 动作的话题名.'))

        # 创建发布者
        motions_topic = self.get_parameter("motions_topic").value
        self.publisher = self.create_publisher(MotionArray, motions_topic, 10)
        self.server = self.create_service(Empty, 'reload_motions', self.reload_motions_callback)

        # 定时器，定期发布消息
        self.read_yaml()
        self.publish_motions()
        self.timer = self.create_timer(10, self.publish_motions)

    def reload_motions_callback(self, _, response):
        self.get_logger().info("重新加载OMS动作...")
        self.read_yaml()
        self.publish_motions()
        self.get_logger().info("OMS动作重新加载完成.")
        return response

    def read_yaml(self):
        try:
            raw_path = self.get_parameter('motion_path').value
            motion_path = resolve_motion_path(raw_path)
            with open(motion_path, 'r', encoding='utf-8') as f:
                self.raw_motions = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"读取 OMS 动作 YAML 文件失败: {e}")
            self.raw_motions = {}

    def publish_motions(self):
        motions_array = MotionArray()

        if not self.raw_motions:
            self.get_logger().warn("没有可用的 OMS 动作.")

        for k, v in self.raw_motions.items():
            try:
                msg = Motion()

                msg.name = k
                msg.rotate = float(v.get("rotate"))
                msg.lift = float(v.get("lift"))
                msg.rotary_servo = float(v.get("rotary_servo"))
                msg.nod_servo = float(v.get("nod_servo"))
                msg.telescopic_servo = float(v.get("telescopic_servo"))
                msg.gripper_servo = float(v.get("gripper_servo"))
                msg.connected_nodes = [ConnectedNode(name=k1, conditions=[KeyBool(key=k2, value=v2)
                                                                          for k2, v2 in v1.get("conditions").items()])
                                       for k1, v1 in v.get("connected_nodes", {}).items()]
                msg.update_status = [KeyBool(key=k1, value=v1) for k1, v1 in v.get("update_status", {}).items()]

                motions_array.motions.append(msg)
                self.get_logger().debug(f"已从参数加载 OMS 动作: {msg.name}")
            except Exception as e:
                self.get_logger().error(f"处理 motion [{k}] 时出错: {e}")
                continue

        self.publisher.publish(motions_array)


def main():
    rclpy.init()

    node = PubMotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
