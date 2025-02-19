import rclpy

import web_message_transform_ros2.msg
from ..util.Singleton import singleton


@singleton
class RobotDataDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.robotData = web_message_transform_ros2.msg.RobotData()

        self.robotDataSub = node.create_subscription(web_message_transform_ros2.msg.RobotData, '/web_transform_node/robot_data', self.robotDataCallback, 10)

    # 读取回调
    def robotDataCallback(self, msg):
        self.robotData = msg

    def getRobotData(self):
        return self.robotData