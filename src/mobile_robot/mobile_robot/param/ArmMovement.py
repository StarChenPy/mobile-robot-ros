import time

from ..popo.Direction import Direction
from typing import TYPE_CHECKING

from ..popo.OmsGoal import OmsGoal

if TYPE_CHECKING:
    from ..service.ArmService import ArmService


OPEN_GRIPPER = 26
OPEN_HALF_GRIPPER = 13
OPEN_GRAPE_GRIPPER = 10

CLOSE_GRIPPER_APPLE = 4
CLOSE_GRIPPER_GRAPE = 1
CLOSE_GRIPPER_BASKET = 15.3


# ------------------ 基础动作与复用动作 ------------------ #


def robot_basket_top(num):
    if num == 1:
        plan_list = [
            OmsGoal(motor_rotary=21, motor_lift=0, servo_rotary=69, servo_telescopic=4),
            OmsGoal(servo_nod=90, sleep=0.5)
        ]
    elif num == 2:
        plan_list = [
            OmsGoal(motor_rotary=7, motor_lift=0, servo_rotary=-97, servo_telescopic=3),
            OmsGoal(servo_nod=90, sleep=0.5)
        ]
    elif num == 3:
        plan_list = [
            OmsGoal(motor_rotary=-19, motor_lift=0, servo_rotary=-69, servo_telescopic=4.5),
            OmsGoal(servo_nod=90, sleep=0.5)
        ]
    else:
        raise ValueError("不正确的框子位置")

    return plan_list


def station_basket_top(direction: Direction, telescopic):
    if direction == Direction.FRONT:
        telescopic -= 11
        rotate = 180
    elif direction == Direction.LEFT:
        rotate = 90
    elif direction == Direction.RIGHT:
        rotate = -90
    else:
        raise ValueError("不支持的方向!")

    plan_list = [
        OmsGoal(motor_lift=0),
        OmsGoal(motor_rotary=rotate, servo_rotary=0, servo_nod=0, servo_telescopic=telescopic),
    ]
    return plan_list


def motion():
    plan_list = [
        OmsGoal(motor_lift=0, servo_rotary=0, servo_nod=-30, servo_telescopic=0, servo_gripper=OPEN_HALF_GRIPPER),
        OmsGoal(motor_rotary=0, motor_lift=16)
    ]
    return plan_list


def motion_apple(arm: 'ArmService'):
    plan_list = [
        OmsGoal(motor_lift=16, servo_rotary=0, servo_nod=-30, servo_telescopic=0, servo_gripper=CLOSE_GRIPPER_APPLE),
        OmsGoal(motor_rotary=0)
    ]
    arm.plan_list(plan_list)


def motion_grape(arm: 'ArmService'):
    plan_list = [
        OmsGoal(motor_lift=16, servo_rotary=0, servo_nod=-30, servo_telescopic=0, servo_gripper=CLOSE_GRIPPER_GRAPE),
        OmsGoal(motor_rotary=0)
    ]
    arm.plan_list(plan_list)


# ------------------ 识别类动作 ------------------ #


def identify_station_fruit(arm: 'ArmService', direction: Direction):
    """
    识别站台上的水果姿态
    """
    if direction == Direction.LEFT:
        rotate = 90
    elif direction == Direction.RIGHT:
        rotate = -90
    else:
        raise ValueError("不支持的方向!")

    arm.plan_once(OmsGoal(motor_rotary=rotate, motor_lift=0, servo_rotary=0, servo_nod=10, servo_telescopic=0))
    time.sleep(1)


def identify_grape(direction: Direction, front_grab_extra_angle=0):
    """
    识别墙上的葡萄姿态
    """
    plan_list = []
    if direction == Direction.FRONT:
        plan_list.append(
            OmsGoal(motor_rotary=180 + front_grab_extra_angle, servo_rotary=-front_grab_extra_angle,
                    servo_nod=-30, servo_gripper=OPEN_HALF_GRIPPER)
        )
    elif direction == Direction.LEFT:
        plan_list.append(
            OmsGoal(motor_rotary=200, servo_rotary=-112, servo_nod=-30, servo_gripper=OPEN_HALF_GRIPPER)
        )
    elif direction == Direction.RIGHT:
        plan_list.append(
            OmsGoal(motor_rotary=-200, servo_rotary=112, servo_nod=-30, servo_gripper=OPEN_HALF_GRIPPER)
        )
    plan_list.append(OmsGoal(motor_lift=33, servo_telescopic=10, sleep=0.5))
    return plan_list


def identify_ground_fruit(arm: 'ArmService', nod_angle=60):
    """
    识别地上的水果姿态
    """
    plan_list = [
        OmsGoal(motor_lift=5, motor_rotary=180, servo_nod=0, servo_telescopic=8),
        OmsGoal(servo_rotary=0, servo_nod=nod_angle)
    ]
    arm.plan_list(plan_list)
    time.sleep(1)


def identify_tree_fruit(arm: 'ArmService', direction: Direction):
    """
    识别苹果树姿态
    """
    arm.lift(0)
    arm.servo_nod(0)
    if direction == Direction.LEFT:
        arm.rotate(180)
        arm.servo_rotary(-90)
    elif direction == Direction.RIGHT:
        arm.rotate(-180)
        arm.servo_rotary(90)
    else:
        raise ValueError("不支持的方向")
    arm.servo_nod(-30)
    arm.lift(28, is_block=False)
    arm.servo_telescopic(0)
    arm.wait_finish()
    time.sleep(1)


# ------------------ 抓取类动作 ------------------ #


def grab_basket_from_robot(arm: 'ArmService', num: int):
    plan_list = robot_basket_top(num) + [
        OmsGoal(motor_lift=10, servo_gripper=OPEN_GRIPPER),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_BASKET),
        OmsGoal(motor_lift=0),
        OmsGoal(servo_rotary=0, servo_nod=0)
    ]
    arm.plan_list(plan_list)


def grab_basket_from_station(direction, lift, telescopic):
    plan_list = [
        OmsGoal(servo_gripper=OPEN_GRIPPER)
    ] + station_basket_top(direction, telescopic) + [
        OmsGoal(motor_lift=lift - 6),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_BASKET, sleep=0.3),
        OmsGoal(servo_gripper=OPEN_GRIPPER, sleep=0.3),
        OmsGoal(motor_lift=lift),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_BASKET),
        OmsGoal(motor_lift=0)
    ]

    return plan_list


def grab_fruit_from_station(arm: 'ArmService', direction: Direction, low=True):
    plan_list = [
        OmsGoal(motor_rotary=90 if direction == Direction.LEFT else -90, motor_lift=6 if low else 0,
                servo_rotary=90, servo_nod=0, servo_telescopic=18, servo_gripper=OPEN_GRIPPER),
        OmsGoal(servo_nod=70, sleep=0.3),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_APPLE, sleep=1),
        OmsGoal(servo_telescopic=0, servo_nod=0)
    ]
    arm.plan_list(plan_list)


def grab_fruit_from_ground(arm: 'ArmService', rotary, telescopic, is_small):
    plan_list = []
    if rotary > 0:
        plan_list.append(
            OmsGoal(motor_rotary=180 + rotary, motor_lift=30,
                    servo_rotary=90 - rotary, servo_nod=90, servo_telescopic=telescopic, servo_gripper=OPEN_HALF_GRIPPER),
        )
    else:
        plan_list.append(
            OmsGoal(motor_rotary=180 + rotary, motor_lift=30,
                    servo_rotary=-90 - rotary, servo_nod=90, servo_telescopic=telescopic, servo_gripper=OPEN_HALF_GRIPPER),
        )
    if is_small:
        plan_list.append(OmsGoal(servo_gripper=CLOSE_GRIPPER_GRAPE, sleep=0.5))
    else:
        plan_list.append(OmsGoal(servo_gripper=CLOSE_GRIPPER_APPLE, sleep=0.5))
    plan_list.append(OmsGoal(motor_lift=0))

    arm.plan_list(plan_list)


def grab_apple_on_tree(arm: 'ArmService', direction: Direction, telescopic: float, is_low: bool, extra_swing_angle: float):
    # 确定旋转方向
    if direction == Direction.LEFT:
        rotate_angle = 90
        servo_rotary = -90 if extra_swing_angle > 0 else 90
    elif direction == Direction.RIGHT:
        rotate_angle = -90
        servo_rotary = 90 if extra_swing_angle > 0 else -90
    else:
        raise ValueError("不支持的方向")

    # 抬升高度
    lift_height = 33 if is_low else 20
    servo_nod = 10 if is_low else 0

    plan_list = [
        OmsGoal(motor_lift=0),
        OmsGoal(motor_rotary=rotate_angle + extra_swing_angle, motor_lift=0,
                servo_rotary=0, servo_nod=servo_nod, servo_telescopic=0, servo_gripper=OPEN_HALF_GRIPPER),
        OmsGoal(motor_lift=lift_height),
        OmsGoal(servo_telescopic=telescopic, servo_nod=45 if extra_swing_angle else servo_nod, sleep=0.3),
        OmsGoal(motor_rotary=rotate_angle),
        OmsGoal(servo_nod=servo_nod, sleep=0.5),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_APPLE, sleep=0.2),
    ]

    # 收回机械臂
    if is_low:
        plan_list.append(OmsGoal(servo_nod=60, servo_telescopic=0))
    else:
        plan_list.append(OmsGoal(servo_nod=90, servo_telescopic=0))

    plan_list.append(OmsGoal(motor_lift=0, servo_rotary=servo_rotary))

    arm.plan_list(plan_list)


def grab_grape_on_wall(direction: Direction, lift_height: float, angle: float, telescopic: float):
    plan_list = []

    if direction == Direction.FRONT:
        plan_list.append(
            OmsGoal(motor_rotary=angle)
        )
        plan_list.append(
            OmsGoal(motor_lift=16, servo_rotary=-angle, servo_nod=90, servo_telescopic=0)
        )
        plan_list.append(
            OmsGoal(motor_lift=lift_height, servo_telescopic=telescopic, servo_gripper=OPEN_GRAPE_GRIPPER, sleep=0.3)
        )
    elif direction == Direction.LEFT:
        plan_list.append(
            OmsGoal(motor_rotary=180)
        )
        plan_list.append(
            OmsGoal(motor_lift=16, servo_rotary=angle - 90, servo_nod=90, servo_telescopic=2)
        )
        plan_list.append(
            OmsGoal(motor_rotary=180 - angle)
        )
        plan_list.append(
            OmsGoal(motor_lift=lift_height, servo_telescopic=telescopic, servo_gripper=OPEN_GRAPE_GRIPPER, sleep=0.3)
        )
    elif direction == Direction.RIGHT:
        plan_list.append(
            OmsGoal(motor_rotary=-180)
        )
        plan_list.append(
            OmsGoal(motor_lift=16, servo_rotary=angle + 90, servo_nod=90, servo_telescopic=2)
        )
        plan_list.append(
            OmsGoal(motor_rotary=-180 - angle)
        )
        plan_list.append(
            OmsGoal(motor_lift=lift_height, servo_telescopic=telescopic, servo_gripper=OPEN_GRAPE_GRIPPER, sleep=0.3)
        )

    plan_list.append(OmsGoal(servo_nod=60, sleep=0.3))
    plan_list.append(OmsGoal(servo_gripper=CLOSE_GRIPPER_GRAPE, sleep=0.3))
    plan_list.append(OmsGoal(servo_telescopic=0))

    return plan_list


# ------------------ 放置类动作 ------------------ #


def put_basket_to_robot(num):
    plan_list = [
        OmsGoal(motor_lift=0, servo_rotary=0, servo_nod=0, servo_gripper=CLOSE_GRIPPER_BASKET)
    ]

    if num == 1:
        plan_list.append(
            OmsGoal(motor_rotary=21, servo_telescopic=0)
        )
        plan_list.append(
            OmsGoal(servo_rotary=69, servo_telescopic=4)
        )
        plan_list.append(
            OmsGoal(motor_lift=3, servo_nod=90, sleep=0.5)
        )
    elif num == 2:
        plan_list.append(
            OmsGoal(motor_rotary=7, servo_telescopic=0)
        )
        plan_list.append(
            OmsGoal(servo_rotary=-97, servo_telescopic=3)
        )
        plan_list.append(
            OmsGoal(motor_lift=3, servo_nod=90, sleep=0.5)
        )
    elif num == 3:
        plan_list.append(
            OmsGoal(motor_rotary=-19, servo_telescopic=0)
        )
        plan_list.append(
            OmsGoal(servo_rotary=-69, servo_telescopic=4.5)
        )
        plan_list.append(
            OmsGoal(motor_lift=3, servo_nod=90, sleep=0.5)
        )
    else:
        raise ValueError("不正确的框子位置")

    plan_list.append(OmsGoal(motor_lift=0, sleep=0.5))
    plan_list.append(OmsGoal(motor_lift=10))
    plan_list.append(OmsGoal(motor_lift=0, servo_gripper=OPEN_GRIPPER))
    plan_list.append(OmsGoal(servo_nod=0))
    return plan_list


def put_basket_to_station(direction: Direction, lift, telescopic):
    plan_list = station_basket_top(direction, telescopic) + [
        OmsGoal(motor_lift=lift),
        OmsGoal(motor_lift=0, servo_gripper=OPEN_GRIPPER)
    ]

    return plan_list


def put_fruit_to_basket(arm: 'ArmService', num, down=False):
    plan_list = [
        OmsGoal(motor_lift=0, servo_telescopic=0),
    ] + robot_basket_top(num)
    if down:
        plan_list.append(OmsGoal(motor_lift=10))
        plan_list.append(OmsGoal(motor_lift=0, servo_gripper=OPEN_GRAPE_GRIPPER))
    else:
        plan_list.append(OmsGoal(servo_gripper=OPEN_HALF_GRIPPER))

    arm.plan_list(plan_list)


def put_fruit_to_ground():
    plan_list = [
        OmsGoal(motor_rotary=180, servo_rotary=0, servo_telescopic=15),
        OmsGoal(motor_lift=30, servo_nod=90),
        OmsGoal(servo_gripper=OPEN_GRIPPER),
    ]
    return plan_list
