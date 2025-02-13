from dataclasses import dataclass
from enum import Enum

from position_motor_ros2.msg import AxisParam, OriginParam


# ============================ 基础 ============================

@dataclass
class Rectangle:
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


# ============================ param/arm_movement.py ============================

@dataclass
class MotorMovement:
    rotate: float
    lift: float


@dataclass
class ServoMotor:
    rotary: float  # 原 gripper_rz
    nod: float  # 原 gripper_ry
    telescopic: float
    gripper: float


@dataclass
class ArmMovement:
    motor: MotorMovement = None
    servo: ServoMotor = None


class MotorCmd(Enum):
    READ_FEEDBACK = 0
    BACK_ORIGIN = 1
    SET_POSITION = 2
    DISABLE = 3


# ============================ param/arm_param.py ============================

# 定义电机参数的数据类
@dataclass
class MotorParams:
    max_value: float  # 最大值 (例如：角度或距离)
    min_value: float  # 最小值 (例如：角度或距离)
    origin_param: OriginParam  # 回原点参数
    ctrl_param: AxisParam  # 位环控制参数
    coding_step: float = None
    coding_dis: float = None


# 定义舵机参数的数据类
@dataclass
class ServoParams:
    pin: int  # 控制引脚编号
    min_value: float  # 最小角度或距离 (可选)
    max_value: float  # 最大角度或距离 (可选)
    zero_duty: float = None  # 零位时的占空比 (可选)
    deg90_duty: float = None  # 90度时的占空比 (可选)
    min_duty: float = None  # 最小占空比 (可选)
    max_duty: float = None  # 最大占空比 (可选)
    itinerary: float = None  # 行程 (角度或距离) (可选)


# ============================ impl/navigation_impl.py ============================

class ResetOdomMode(Enum):
    RESET_ALL = 0
    RESET_POSE = 1
    RESET_YAW = 2


class BaseMotionMode(Enum):
    QUERY = 0
    STOP = 1
    LINE = 2
    ROTATE = 3


# ============================ impl/revise_impl.py ============================

class CorrectiveSensor(Enum):
    PING0 = 0
    PING1 = 1
    IR = 2
    PING = 3


# ============================ impl/vision_impl.py ============================

@dataclass
class MnnResult:
    classId: str
    confidence: float
    box: Rectangle


# ============================ param/navigation_param.py ============================

@dataclass
class Corrective:
    sensor: CorrectiveSensor
    distance: float


@dataclass
class NavigationPoint:
    pose: Pose
    corrective: Corrective = None
