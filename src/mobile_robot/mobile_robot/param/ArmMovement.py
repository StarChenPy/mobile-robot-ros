from ..service.ArmService import ArmService
from ..util.Logger import Logger


class ArmMovement:
    def __init__(self, node):
        self.logger = Logger()

        self.node = node
        self.arm = ArmService(node)

    def open_gripper(self):
        self.arm.gripper_servo(24)

    def close_gripper(self, size):
        self.arm.gripper_servo(size)

    def motion(self):
        self.arm.rotate(0, 20, False)
        self.arm.lift(10, 20, False)
        self.arm.rotary_servo(0)
        self.arm.nod_servo(0)
        self.arm.telescopic_servo(0)
        self.arm.gripper_servo(0)

    def identify_station_fruit(self):
        self.arm.rotate(-90, 40, False)
        self.arm.lift(0, 20, False)
        self.arm.telescopic_servo(0)
        self.arm.nod_servo(45)

    def ready_to_grab_fruit_from_station(self):
        self.arm.rotate(-90, 40, False)
        self.arm.lift(0, 20, False)
        self.arm.telescopic_servo(0)
        self.arm.nod_servo(45)
        self.arm.rotary_servo(90)
