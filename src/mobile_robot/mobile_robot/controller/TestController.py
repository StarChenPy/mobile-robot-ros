import rclpy

from ..param import ArmMovement
from ..popo.IdentifyResult import IdentifyResult
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TestController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.node = node
        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)


    def find_grape(self) -> IdentifyResult | None:
        grape = ["yellow_grapes", "green_grapes", "purple_grapes"]
        identify = self.vision.get_onnx_identify()
        for i in identify:
            i: IdentifyResult
            if i.class_id not in grape:
                continue
            return i
        return None

    def run(self):
        self.robot.with_robot_connect()
        # self.arm.back_origin()

        ArmMovement.identify_ground_fruit(self.arm)

        while True:
            input("等待...")
            grape = self.find_grape()
            if grape:
                center = grape.box.get_rectangle_center()
                x_dis = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, 0.41)
                photo_telescopic_len = 0.25

                telescopic_len = Math.calculate_hypotenuse(photo_telescopic_len, x_dis)
                print("x_dis:", x_dis)
                self.arm.telescopic_servo((telescopic_len - photo_telescopic_len) * 100 + 6)

                # rotary_angle = -Math.calculate_right_triangle_angle(x_dis, telescopic_len)
                # print(rotary_angle, telescopic_len)
                # if 180 + rotary_angle > 200:
                #     self.arm.rotate(-180 + rotary_angle, 40, False)
                #     self.arm.rotary_servo(-90 - rotary_angle)
                #     self.arm.rotary_servo(-90 - rotary_angle)
                #     self.arm.rotary_servo(-90 - rotary_angle)
                # else:
                #     self.arm.rotate(180 + rotary_angle, 40, False)
                #     self.arm.rotary_servo(90 - rotary_angle)
                #     self.arm.rotary_servo(90 - rotary_angle)
                #     self.arm.rotary_servo(90 - rotary_angle)

                rotary_angle = -Math.calculate_right_triangle_angle(x_dis, telescopic_len)
                print("rotary_angle:", rotary_angle)
                if 180 + rotary_angle > 200:
                    self.arm.rotary_servo(-90 - rotary_angle)
                else:
                    self.arm.rotary_servo(90 - rotary_angle)
                self.arm.rotate(180 + rotary_angle, 40)

                input("123")

                ArmMovement.identify_ground_fruit(self.arm)
