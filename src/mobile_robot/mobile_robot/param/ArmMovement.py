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
CLOSE_GRIPPER_BASKET = 15


def robot_basket_top(num):
    if num == 1:
        plan_list = [
            OmsGoal(motor_rotary=20, motor_lift=0, servo_rotary=69, servo_nod=0, servo_telescopic=3),
            OmsGoal(servo_nod=90)
        ]
    elif num == 2:
        plan_list = [
            OmsGoal(motor_rotary=0, motor_lift=0, servo_rotary=-85, servo_nod=0, servo_telescopic=2),
            OmsGoal(servo_nod=95)
        ]
    elif num == 3:
        plan_list = [
            OmsGoal(motor_rotary=-23, motor_lift=0, servo_rotary=-65, servo_nod=0, servo_telescopic=4),
            OmsGoal(servo_nod=90)
        ]
    else:
        raise ValueError("不正确的框子位置")

    return plan_list


def station_basket_top(arm: 'ArmService', direction: Direction, telescopic, rotate_offset):
    if direction == Direction.LEFT:
        rotate = 90
    elif direction == Direction.RIGHT:
        rotate = -90
    else:
        raise ValueError("不支持的方向!")
    rotate -= rotate_offset

    arm.lift(0)
    arm.rotate(rotate, is_block=False)
    arm.servo_nod(0)
    arm.servo_rotary(0)
    arm.servo_telescopic(telescopic)
    arm.wait_finish()


def end(arm: 'ArmService'):
    plan_list = [
        OmsGoal(motor_rotary=0, motor_lift=0, servo_rotary=0, servo_telescopic=0, servo_gripper=OPEN_HALF_GRIPPER),
        OmsGoal(servo_nod=90)
    ]
    arm.plan_list(plan_list)


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


def identify_station_fruit(arm: 'ArmService', direction: Direction):
    """
    识别站台上的水果姿态
    """
    arm.lift(0, is_block=False)
    arm.rotate(90 if direction == Direction.LEFT else -90, is_block=False)
    arm.servo_telescopic(0)
    arm.servo_nod(10)
    arm.wait_finish()
    time.sleep(1)


def identify_grape(arm: 'ArmService', direction: Direction):
    """
    识别墙上的葡萄姿态
    """
    plan_list = []
    if direction == Direction.LEFT:
        plan_list.append(
            OmsGoal(motor_rotary=200, servo_rotary=-108, servo_nod=-25, servo_telescopic=0, servo_gripper=OPEN_HALF_GRIPPER)
        )
    elif direction == Direction.RIGHT:
        plan_list.append(
            OmsGoal(motor_rotary=-200, servo_rotary=112, servo_nod=-25, servo_telescopic=0, servo_gripper=OPEN_HALF_GRIPPER)
        )
    plan_list.append(OmsGoal(motor_lift=29, servo_telescopic=10))
    arm.plan_list(plan_list)
    time.sleep(1)


def identify_ground_fruit(arm: 'ArmService'):
    """
    识别地上的水果姿态
    """
    plan_list = [
        OmsGoal(motor_lift=5, motor_rotary=180, servo_nod=0, servo_telescopic=3),
        OmsGoal(servo_nod=60, servo_rotary=90),
        OmsGoal(servo_rotary=180)
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
    arm.lift(22, is_block=False)
    arm.servo_telescopic(0)
    arm.wait_finish()
    time.sleep(1)


def grab_basket_from_robot(arm: 'ArmService', num: int):
    plan_list = robot_basket_top(num) + [
        OmsGoal(motor_lift=10, servo_gripper=OPEN_GRIPPER),
        OmsGoal(motor_lift=0, servo_gripper=CLOSE_GRIPPER_BASKET),
        OmsGoal(servo_rotary=0, servo_nod=0, servo_telescopic=18)
    ]
    arm.plan_list(plan_list)


def grab_basket_from_station(arm: 'ArmService', direction, lift, telescopic, angle):
    arm.servo_gripper(OPEN_GRIPPER)
    arm.servo_nod(0)
    station_basket_top(arm, direction, telescopic, angle)

    arm.lift(lift)
    arm.servo_gripper(CLOSE_GRIPPER_BASKET)
    arm.lift(0)


def grab_fruit_from_station(arm: 'ArmService', direction: Direction, low=True):
    plan_list = [
        OmsGoal(motor_rotary=90 if direction == Direction.LEFT else -90, motor_lift=6 if low else 0,
                servo_rotary=90, servo_nod=0, servo_telescopic=18, servo_gripper=OPEN_GRIPPER),
        OmsGoal(servo_nod=70, sleep=0.3),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_APPLE),
        OmsGoal(servo_telescopic=0, servo_nod=0)
    ]
    arm.plan_list(plan_list)


def grab_fruit_from_ground(arm: 'ArmService', rotary, telescopic, is_grape):
    plan_list = [
        OmsGoal(motor_rotary=180 + rotary, motor_lift=29,
                servo_rotary=90 - rotary, servo_nod=90, servo_telescopic=telescopic, servo_gripper=OPEN_HALF_GRIPPER),
    ]
    if is_grape:
        plan_list.append(OmsGoal(servo_gripper=CLOSE_GRIPPER_GRAPE))
    else:
        plan_list.append(OmsGoal(servo_gripper=CLOSE_GRIPPER_APPLE))
    plan_list.append(OmsGoal(motor_lift=0))

    arm.plan_list(plan_list)


def put_basket_to_robot(num):
    plan_list = [
        OmsGoal(motor_lift=0, servo_rotary=0, servo_nod=0, servo_gripper=CLOSE_GRIPPER_BASKET)
    ]

    if num == 1:
        plan_list.append(
            OmsGoal(motor_rotary=20, servo_rotary=72, servo_telescopic=5.5)
        )
        plan_list.append(
            OmsGoal(motor_lift=3, servo_nod=90)
        )
    elif num == 2:
        plan_list.append(
            OmsGoal(motor_rotary=0, servo_rotary=-85, servo_telescopic=2)
        )
        plan_list.append(
            OmsGoal(motor_lift=3, servo_nod=95)
        )
    elif num == 3:
        plan_list.append(
            OmsGoal(motor_rotary=-20, servo_rotary=-68, servo_telescopic=5)
        )
        plan_list.append(
            OmsGoal(motor_lift=3, servo_nod=90)
        )
    else:
        raise ValueError("不正确的框子位置")

    plan_list.append(OmsGoal(motor_lift=0))
    plan_list.append(OmsGoal(motor_lift=10))
    plan_list.append(OmsGoal(motor_lift=0, servo_gripper=OPEN_GRIPPER))
    plan_list.append(OmsGoal(servo_nod=0))
    return plan_list


def put_basket_to_station(arm: 'ArmService', direction: Direction, lift):
    station_basket_top(arm, direction, 5, 0)

    arm.lift(lift)
    arm.servo_gripper(OPEN_GRIPPER)
    arm.lift(0)


def put_fruit_to_basket(arm: 'ArmService', num, down=False):
    plan_list = [
        OmsGoal(motor_lift=0, servo_telescopic=0),
    ] + robot_basket_top(num)
    if down:
        plan_list.append(OmsGoal(motor_lift=10))
        plan_list.append(OmsGoal(motor_lift=0, servo_gripper=OPEN_HALF_GRIPPER))
    else:
        plan_list.append(OmsGoal(servo_gripper=OPEN_HALF_GRIPPER))

    arm.plan_list(plan_list)


def put_fruit_to_ground(arm: 'ArmService'):
    plan_list = [
        OmsGoal(motor_rotary=180, motor_lift=30, servo_rotary=0, servo_nod=90, servo_telescopic=15),
        OmsGoal(servo_gripper=OPEN_GRIPPER),
    ]
    arm.plan_list(plan_list)


def grab_apple_on_tree(arm: 'ArmService', direction: Direction, telescopic: float, is_low: bool):
    # 确定旋转方向
    if direction == Direction.LEFT:
        rotate_angle = 90
    elif direction == Direction.RIGHT:
        rotate_angle = -90
    else:
        raise ValueError("不支持的方向")

    # 抬升高度
    lift_height = 30 if is_low else 20

    plan_list = [
        OmsGoal(motor_rotary=rotate_angle, motor_lift=0,
                servo_rotary=0, servo_nod=0, servo_telescopic=0, servo_gripper=OPEN_HALF_GRIPPER),
        OmsGoal(motor_lift=lift_height),
        OmsGoal(servo_telescopic=telescopic),
        OmsGoal(servo_gripper=CLOSE_GRIPPER_APPLE, sleep=0.2),
    ]

    # 收回机械臂
    if is_low:
        plan_list.append(OmsGoal(servo_nod=60, servo_telescopic=0))
    else:
        plan_list.append(OmsGoal(servo_nod=90, servo_telescopic=0))

    plan_list.append(OmsGoal(motor_lift=0, servo_rotary=90, servo_nod=0))

    arm.plan_list(plan_list)


def grab_grape_on_wall(arm: 'ArmService', direction: Direction, lift_height: float, angle: float):
    plan_list = [
        OmsGoal(servo_telescopic=0)
    ]

    if direction == Direction.LEFT:
        plan_list.append(
            OmsGoal(motor_lift=lift_height, servo_rotary=-(90 - angle),
                    servo_nod=90, servo_telescopic=0, servo_gripper=OPEN_GRAPE_GRIPPER))
        plan_list.append(
            OmsGoal(motor_rotary=180 - angle)
        )
    if direction == Direction.RIGHT:
        plan_list.append(
            OmsGoal(motor_lift=lift_height, servo_rotary=90 - angle,
                    servo_nod=90, servo_telescopic=0, servo_gripper=OPEN_GRAPE_GRIPPER))
        plan_list.append(
            OmsGoal(motor_rotary=angle - 180)
        )

    plan_list.append(OmsGoal(servo_telescopic=10))
    plan_list.append(OmsGoal(servo_nod=60, sleep=0.3))
    plan_list.append(OmsGoal(servo_gripper=CLOSE_GRIPPER_GRAPE, sleep=0.3))

    if direction == Direction.LEFT:
        plan_list.append(OmsGoal(motor_rotary=180))
    elif direction == Direction.RIGHT:
        plan_list.append(OmsGoal(motor_rotary=-180))

    arm.plan_list(plan_list)
