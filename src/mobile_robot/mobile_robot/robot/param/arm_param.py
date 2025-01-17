from dataclasses import dataclass
from enum import Enum

from position_motor_ros2 import msg


class MotorCmd(Enum):
    READ_FEEDBACK = 0
    BACK_ORIGIN = 1
    SET_POSITION = 2
    DISABLE = 3


# 定义电机参数的数据类
@dataclass
class MotorParams:
    max_value: float  # 最大值 (例如：角度或距离)
    min_value: float  # 最小值 (例如：角度或距离)
    origin_param: msg.OriginParam  # 回原点参数
    ctrl_param: msg.AxisParam  # 位环控制参数
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


# 定义舵机枚举
class Servo(Enum):
    ROTARY_SERVO = ServoParams(
        pin=4,
        min_value=-40,
        max_value=90,
        zero_duty=24.4,
        deg90_duty=12.5
    )  # 卡爪舵机 旋转
    NOD_SERVO = ServoParams(
        pin=1,
        min_value=-95,
        max_value=90,
        zero_duty=28.8,
        deg90_duty=9.9
    )  # 卡爪舵机 点头(角度)
    TELESCOPIC_SERVO = ServoParams(
        pin=2,
        min_value=-100,  # 并没有最小最大值，随便写的
        max_value=100,
        min_duty=12,
        max_duty=49,
        itinerary=15
    )  # 卡爪舵机 伸缩
    GRIPPER_SERVO = ServoParams(
        pin=3,
        min_value=0,
        max_value=24.5,
        min_duty=12.5,
        max_duty=41.0,
        itinerary=10.5
    )  # 卡爪舵机 夹合


# 定义电机枚举
class Motor(Enum):
    ROTATE = MotorParams(
        max_value=180.0,
        min_value=-90.0,
        origin_param=msg.OriginParam(
            back_origin_vel=40.0,
            find_origin_cnt=2,
            approix_origin_vel=7.0,
            approix_origin_step=150.0,
            origin_mode=1,
            origin_max_step=1000
        ),
        ctrl_param=msg.AxisParam(
            kp=0.25,
            ti=0.0,
            td=0.0,
            max_acc=800.0,
            low_pass=0.75,
            ek=5.0,
            steady_clk=10
        )
    )  # 旋转电机
    LIFT = MotorParams(
        max_value=28.0,
        min_value=0.0,
        origin_param=msg.OriginParam(
            back_origin_vel=75.0,
            find_origin_cnt=2,
            approix_origin_vel=10.0,
            approix_origin_step=150.0,
            origin_mode=0,
            origin_max_step=3000
        ),
        ctrl_param=msg.AxisParam(
            kp=0.20,
            ti=0.0,
            td=0.0,
            max_acc=600.0,
            low_pass=0.75,
            ek=10.0,
            steady_clk=10
        ),
        coding_step=3000,
        coding_dis=30
    )  # 举升电机
