import enum


class Servo(enum.Enum):
    NOD = 1  # 卡爪舵机 点头(角度)
    TELESCOPIC = 2  # 卡爪舵机 伸缩
    GRIPPER = 3  # 卡爪舵机 夹合
    ROTARY = 4  # 卡爪舵机 旋转
