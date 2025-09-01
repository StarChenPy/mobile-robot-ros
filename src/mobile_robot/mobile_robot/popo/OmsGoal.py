import dataclasses


@dataclasses.dataclass
class OmsGoal:
    motor_rotary: float = None
    motor_lift: float = None
    servo_rotary: float = None
    servo_nod: float = None
    servo_telescopic: float = None
    servo_gripper: float = None
    sleep: float = 0
