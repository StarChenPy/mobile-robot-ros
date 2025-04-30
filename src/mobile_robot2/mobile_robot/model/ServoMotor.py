from dataclasses import dataclass


@dataclass
class ServoMotor:
    rotary: float  # 原 gripper_rz
    nod: float  # 原 gripper_ry
    telescopic: float
    gripper: float
