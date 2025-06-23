import enum

import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from base_motion_ros2.srv import BaseMotion
from corn_robot_interfaces.action import RobotMotion


class MotionMode(enum.Enum):
    QUERY = 0
    STOP = 1
    LINE = 2
    ROTATE = 3


class RobotMotionNode(rclpy.node.Node):
    def __init__(self):
        """基础运动节点"""
        super().__init__('robot_motion_node')
        self.get_logger().info('基础运动节点 已启动.')

        self.declare_parameter('line_kp', 1.8)
        self.declare_parameter('line_ti', 0.0)
        self.declare_parameter('line_td', 0.0)
        self.declare_parameter('line_max_vel', 1.0)
        self.declare_parameter('line_max_acc', 3.0)
        self.declare_parameter('line_low_pass', 0.8)
        self.declare_parameter('line_ek', 0.02)
        self.declare_parameter('line_steady_clk', 5)

        self.declare_parameter('rotate_kp', 2.8)
        self.declare_parameter('rotate_ti', 0.0)
        self.declare_parameter('rotate_td', 0.0001)
        self.declare_parameter('rotate_max_vel', 180.0)
        self.declare_parameter('rotate_max_acc', 600.0)
        self.declare_parameter('rotate_low_pass', 0.7)
        self.declare_parameter('rotate_ek', 1.0)
        self.declare_parameter('rotate_steady_clk', 10)
        self.declare_parameter('rotate_planner_acc', 600.0)
        self.declare_parameter('rotate_planner_decel', 300.0)
        self.declare_parameter('robot_motion_action', 'robot_motion',
                               ParameterDescriptor(description='基础运动服务名.'))

        # 动作服务器
        action_name = self.get_parameter("robot_motion_action").value
        self.action = ActionServer(self,
                                   RobotMotion,
                                   action_name,
                                   execute_callback=self.execute_callback,
                                   cancel_callback=self.cancel_callback)

        # 客户端
        self.hb_motion_client = self.create_client(BaseMotion, '/base_motion')

        self.get_logger().info("等待 /base_motion 服务可用...")
        self.hb_motion_client.wait_for_service()
        self.get_logger().info("/base_motion 服务已可用.")

    async def cancel_callback(self, _):
        """处理机器人运动动作取消请求"""
        self.get_logger().info("接收到机器人运动取消请求.")
        req = self.create_motion_request(MotionMode.STOP, 0, 0)
        await self.hb_motion_client.call_async(req)
        return rclpy.action.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """处理机器人运动动作请求"""
        request: RobotMotion.Goal = goal_handle.request
        self.get_logger().info(f"接收到机器人运动请求: {request}")

        await self.wait_finish(True)

        # 旋转运动请求
        if request.angle != 0:
            result = await self.execute_motion(goal_handle, MotionMode.ROTATE, request.angle, request.speed * 100)
            if result is not None:
                return result

        # 直线运动请求
        if request.distance != 0:
            result = await self.execute_motion(goal_handle, MotionMode.LINE, request.distance, request.speed)
            if result is not None:
                return result

        goal_handle.succeed()
        self.get_logger().info("机器人运动请求已完成，返回结果.")
        return RobotMotion.Result(success=True, message="机器人运动完成!")

    async def wait_finish(self, start=False):
        """
        等待基础运动完成
        {'feedback': {'motion_mode': 0, 'motion_status': 2, 'error': 0.0}, 'success': True}
        motion_mode: 0:空闲 1:直线 2:旋转
        motion_status: 0:空闲 1:运行中 2:完成
        """
        while rclpy.ok():
            req = self.create_motion_request(MotionMode.QUERY, 0, 0)
            result = await self.hb_motion_client.call_async(req)

            if result is None:
                continue

            feedback = result.feedback
            if feedback.motion_mode == 0 and (feedback.motion_status == 2 or (feedback.motion_status == 0 and start)):
                self.get_logger().info("机器人运动已结束.")
                return

    async def execute_motion(self, goal_handle: ServerGoalHandle, mode: MotionMode, set_point, speed):
        """封装 单次运动请求 + 等待结束 逻辑"""
        req = self.create_motion_request(mode, set_point, speed)
        self.get_logger().info(f"开始 {mode.name} 运动")
        result: BaseMotion.Response = await self.hb_motion_client.call_async(req)

        if result is None:
            req = self.create_motion_request(MotionMode.STOP, 0, 0)
            self.get_logger().warn(f'/base_motion {mode.name} 服务调用失败')
            await self.hb_motion_client.call_async(req)
            goal_handle.abort()
            return RobotMotion.Result(success=False, message=f'/base_motion {mode.name} 服务调用失败')
        elif result.success is False:
            self.get_logger().warn(f'/base_motion {mode.name} 拒绝服务: 运动状态为 {result.feedback.motion_status}')
            goal_handle.abort()
            return RobotMotion.Result(success=False,
                                      message=f'/base_motion {mode.name} 拒绝服务: 运动状态为 {result.feedback.motion_status}')

        self.get_logger().info(f"/base_motion {mode.name} 请求已完成，等待运动结束.")
        await self.wait_finish()
        return None  # 表示执行成功

    def create_motion_request(self, mode: MotionMode, set_point, speed) -> BaseMotion.Request:
        """构建 BaseMotion 请求"""
        req = BaseMotion.Request()
        req.motion_mode = mode.value
        req.set_point = float(set_point)

        req.line_param.kp = self.get_parameter('line_kp').value
        req.line_param.ti = self.get_parameter('line_ti').value
        req.line_param.td = self.get_parameter('line_td').value
        req.line_param.max_vel = float(speed)
        req.line_param.max_acc = self.get_parameter('line_max_acc').value
        req.line_param.low_pass = self.get_parameter('line_low_pass').value
        req.line_param.ek = self.get_parameter('line_ek').value
        req.line_param.steady_clk = self.get_parameter('line_steady_clk').value

        if mode == MotionMode.ROTATE:
            req.rotate_param.kp = self.get_parameter('rotate_kp').value
            req.rotate_param.ti = self.get_parameter('rotate_ti').value
            req.rotate_param.td = self.get_parameter('rotate_td').value
            req.rotate_param.max_vel = float(speed)
            req.rotate_param.max_acc = self.get_parameter('rotate_max_acc').value
            req.rotate_param.low_pass = self.get_parameter('rotate_low_pass').value
            req.rotate_param.ek = self.get_parameter('rotate_ek').value
            req.rotate_param.steady_clk = self.get_parameter('rotate_steady_clk').value
            req.rotate_param.planner_vel = float(speed)
            req.rotate_param.planner_acc = self.get_parameter('rotate_planner_acc').value
            req.rotate_param.planner_decel = self.get_parameter('rotate_planner_decel').value
        else:
            req.rotate_param.kp = 1.8
            req.rotate_param.ti = 0.0
            req.rotate_param.td = 0.0001
            req.rotate_param.max_vel = 180.0
            req.rotate_param.max_acc = 600.0
            req.rotate_param.low_pass = 0.8
            req.rotate_param.ek = 1.0
            req.rotate_param.steady_clk = 2

        return req


def main():
    rclpy.init()
    node = RobotMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
