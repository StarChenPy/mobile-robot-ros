import rclpy
import time

from revise_interfaces.msg import ReviseData  # 自定义控制消息
from mnn_interfaces.srv import MnnService # mnn 服务

class Controls:
    __controls_parameter = None
    __mnn_prediction_result = None
    __result_dict = []

    def __init__(self, ros: rclpy.Node):
        self.__ros_node = ros
        # 发布控制数据 话题
        self.__controls_pub = self.__ros_node.create_publisher(ReviseData, '/robot/correct_cmd', 1)
        # 获取控制状态 话题
        self.__controls_sub = self.__ros_node.create_subscription(ReviseData, '/robot/correct_cmd', self.controls_parameter_callback, 10)
        # mnn 服务
        self.mnn_client = self.__ros_node.create_client(MnnService, '/mnn/mnn_cmd')

        self.__ros_node.get_logger().info('[矫正控制接口] 初始化完成.')

    # 控制参数状态回调
    def controls_parameter_callback(self, msg):
        self.__controls_parameter = msg

    # 控制状态发布
    def pub_controls_parameter(self, x, y, yaw, sensor_selection=0, start=False):
        msg_pub = ReviseData()
        msg_pub.set_revise_x = x
        msg_pub.set_revise_y = y
        msg_pub.set_revise_yaw = yaw
        msg_pub.sensor_selection = sensor_selection
        msg_pub.start = start
        msg_pub.status = 1
        self.__controls_pub.publish(msg_pub)

    # 等待修正结束
    def wait_controls_end(self):
        while self.__controls_parameter.status != 0 :
            start = self.__controls_parameter.start
            status = self.__controls_parameter.status
            self.__ros_node.get_logger().info(f"状态: {start}, 时间: {status}")
            time.sleep(0.5)
        self.__ros_node.get_logger().info("已结束矫正！")


    # mnn 服务回调
    def mnn_result_callback(self, result_future):
        response = result_future.result()

        if not response.success:
            self.__ros_node.get_logger().info("无结果！")
            return

        self.__result_dict = []
        for label, confidence, x1, y1, x2, y2 in zip(response.label, response.confidence, response.xmin, response.ymin, response.xmax, response.ymax):
            self.__result_dict.append({
                "label": label,
                "confidence": confidence,
                "box": [x1, y1, x2, y2]})

    # 发送mnn请求
    def send_mnn_request(self, cmd, data):
        while rclpy.ok() and self.mnn_client.wait_for_service(1) == False:
            self.__ros_node.get_logger().info("等待服务端上线....")
            
        request = MnnService.Request()
        request.cmd = cmd
        request.data = data
        self.mnn_client.call_async(request).add_done_callback(self.mnn_result_callback)

    # 获取mnn推理结果
    def mnn_result(self):
        while not self.__result_dict:
            time.sleep(0.1)
        self.__ros_node.get_logger().debug(f"MNN 识别结果: {self.__result_dict}")

        result_dict = self.__result_dict.copy()
        self.__result_dict = []
        return result_dict
        