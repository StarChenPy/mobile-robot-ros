import rclpy

from rclpy.node import Node
from std_srvs.srv import Trigger

from .robot.param import robot_param
from .robot.impl import arm_impl


class MinimalService(Node):

    def __init__(self):
        super().__init__('arm_reset')

        self.arm = arm_impl.ArmImpl(self)
        self.create_service(Trigger, 'arm/reset', self.arm_reset_callback)

    def arm_reset_callback(self, _, response: Trigger.Response):
        self.get_logger().info("[机械臂] 开始复位！")
        # 升降电机回原点
        self.arm.ctrl_lift_motor(arm_impl.RotateMotorCmd.BACK_ORIGIN)  # 回原点
        self.arm.ctrl_lift_motor(arm_impl.RotateMotorCmd.SET_POSITION, 0.5, 50) # 升降控制

        # 旋转电机回原点
        self.arm.ctrl_rotate_motor(arm_impl.RotateMotorCmd.BACK_ORIGIN)  # 回原点

        self.arm.telescopic_servo(robot_param.TelescopicParams.复位.value) # 伸缩控制
        self.arm.nod_servo(robot_param.GripperRyParams.复位.value) # 摆臂控制
        self.arm.gripper_servo(robot_param.GripperParams.复位.value) # 卡爪控制
        self.arm.rotary_servo(robot_param.GripperRzParams.复位.value) # 旋臂控制

        self.get_logger().info("[机械臂] 复位完成！")

        response.success = True
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()