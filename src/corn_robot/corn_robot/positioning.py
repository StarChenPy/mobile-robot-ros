import rclpy.node
import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from chassis_msgs.srv import ResetOdom
from web_message_transform_ros2.msg import Pose


class PositioningNode(rclpy.node.Node):
    """
    读取map坐标系，并发布到user/odom中
    """
    def __init__(self):
        super().__init__("positioning")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.service = self.create_client(ResetOdom, '/chassis/reset_odom')
        self.timer = self.create_timer(3, self.publish_user_odom)

    def publish_user_odom(self):
        pose = self.get_map_tf_pose()

        if pose:
            req = ResetOdom.Request()
            req.clear_mode = 0
            req.x = pose.x
            req.y = pose.y
            req.theta = pose.w
            self.service.call_async(req)

    def get_map_tf_pose(self) -> Pose | None:
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.x = trans.transform.translation.x
            pose.y = trans.transform.translation.y


            _, _, yaw = euler_from_quaternion([
                trans.transform.rotation.x, trans.transform.rotation.y,
                trans.transform.rotation.z, trans.transform.rotation.w
            ])
            pose.w = yaw
            return pose
        except Exception as e:
            self.get_logger().warn(f'获取 TF 失败: {e}')
            return None

def main():
    rclpy.init()

    node = PositioningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()