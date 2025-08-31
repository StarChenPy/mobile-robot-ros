import rclpy.node
from std_srvs.srv import Trigger

from ..util.Logger import Logger


class JXNav2Dao:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()
        self.node = node
        self.service = node.create_client(Trigger, "/start_fixed_nav")

    def call_service(self):
        req = Trigger.Request()
        future = self.service.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()