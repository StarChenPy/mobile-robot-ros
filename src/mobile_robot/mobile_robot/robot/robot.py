import time
import param.robot_param as rp

from rclpy.node import Node

from impl.navigation_impl import NavigationImpl
from impl.io_impl import IoImpl
from impl.arm_impl import ArmImpl
from impl.vision_impl import CameraImpl


class MobileRobot:
    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        self.navigation = NavigationImpl(node)
        self.arm = ArmImpl(node)
        self.io = IoImpl(node)
        self.camera = CameraImpl(node)

    # 等待启动按键被按下
    def with_start_button(self):
        self.io.set_led(False)
        self.__logger.info('等待按键按下!')
        # 等待按键按下
        while True:
            val = self.io.get_start_button()
            if not val:
                self.__logger.info('按键按下!')
                self.io.set_led(True)
                break
            time.sleep(0.2)

    def arm_reset(self):
        self.__logger.info("OMS 开始复位！")
        # 升降电机回原点
        self.arm.lift_motor('back_origin')  # 回原点
        self.arm.lift_motor_wait_finished()  # 等待运动完成
        self.arm.lift_motor('set_position', 0.5, 70) # 升降控制
        # 旋转电机回原点
        self.arm.rotate_motor('back_origin')  # 回原点
        self.arm.rotate_motor_wait_finished()  # 等待运动完成

        self.arm.telescopic(rp.TelescopicParams.复位) # 伸缩控制
        self.arm.gripper_ry(rp.GripperRyParams.复位) # 摆臂控制
        self.arm.gripper(rp.GripperParams.复位) # 卡爪控制
        self.arm.gripper_rz(rp.GripperRzParams.复位) # 旋臂控制
        time.sleep(1)
        self.__logger.info("OMS 复位完成！")
