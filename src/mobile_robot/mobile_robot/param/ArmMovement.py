import time

from ..popo.Direction import Direction
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..service.ArmService import ArmService


def open_gripper(arm: 'ArmService'):
    arm.gripper_servo(26)


def open_half_gripper(arm: 'ArmService'):
    arm.gripper_servo(14)


def open_one_third_gripper(arm: 'ArmService'):
    arm.gripper_servo(8)


def close_gripper_apple(arm: 'ArmService'):
    arm.gripper_servo(4)
    time.sleep(0.2)


def close_gripper_grape(arm: 'ArmService'):
    arm.gripper_servo(1)
    time.sleep(0.2)


def close_gripper_basket(arm: 'ArmService'):
    arm.gripper_servo(15)
    time.sleep(0.2)


def robot_basket_top(arm: 'ArmService', num):
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    if num == 1:
        arm.rotate(20, is_block=False)
        arm.telescopic_servo(3)
        arm.rotary_servo(69)
        arm.wait_finish()
        arm.nod_servo(90)
    elif num == 2:
        arm.rotate(0, is_block=False)
        arm.rotary_servo(-85)
        arm.telescopic_servo(2)
        arm.wait_finish()
        arm.nod_servo(95)
    elif num == 3:
        arm.rotate(-23, is_block=False)
        arm.rotary_servo(-65)
        arm.telescopic_servo(4)
        arm.wait_finish()
        arm.nod_servo(90)
    else:
        raise ValueError("不正确的框子位置")


def station_basket_top(arm: 'ArmService', direction: Direction):
    if direction == Direction.LEFT:
        rotate = 90
    elif direction == Direction.RIGHT:
        rotate = -90
    else:
        raise ValueError("不支持的方向!")

    arm.lift(0)
    arm.rotate(rotate, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(0)
    arm.telescopic_servo(5)
    arm.wait_finish()


def end(arm: 'ArmService'):
    arm.rotate(0, is_block=False)
    arm.lift(0)
    arm.rotary_servo(0)
    arm.nod_servo(90)
    arm.telescopic_servo(0)
    open_half_gripper(arm)
    arm.wait_finish()


def motion(arm: 'ArmService'):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(-20)
    arm.telescopic_servo(0)
    open_half_gripper(arm)
    arm.wait_finish()
    arm.lift(16)


def motion_apple(arm: 'ArmService'):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(-20)
    arm.telescopic_servo(0)
    close_gripper_apple(arm)
    arm.wait_finish()
    arm.lift(18)


def motion_grape(arm: 'ArmService'):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(-20)
    arm.telescopic_servo(0)
    close_gripper_grape(arm)
    arm.wait_finish()
    arm.lift(18)


def top_180(arm: 'ArmService'):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.wait_finish()


def identify_station_fruit(arm: 'ArmService', direction: Direction):
    arm.lift(0, is_block=False)
    if direction == Direction.LEFT:
        arm.rotate(90, is_block=False)
    elif direction == Direction.RIGHT:
        arm.rotate(-90, is_block=False)
    arm.telescopic_servo(0)
    arm.nod_servo(90)
    open_gripper(arm)
    arm.wait_finish()
    arm.nod_servo(60)
    time.sleep(1)


def identify_grape(arm: 'ArmService', direction: Direction):
    if direction == Direction.LEFT:
        arm.rotate(190)
        arm.rotary_servo(-96)
    elif direction == Direction.RIGHT:
        arm.rotate(-190)
        arm.rotary_servo(100)
    arm.telescopic_servo(0)
    open_gripper(arm)
    arm.nod_servo(10)
    arm.lift(35)
    arm.telescopic_servo(10)


def grab_basket_from_robot(arm: 'ArmService', num: int):
    robot_basket_top(arm, num)

    open_gripper(arm)

    arm.lift(10)
    close_gripper_basket(arm)
    arm.lift(0)
    arm.telescopic_servo(17)
    arm.rotary_servo(0)
    arm.nod_servo(0)


def grab_fruit_from_station(arm: 'ArmService'):
    arm.rotate(90, is_block=False)
    arm.lift(4, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(90)
    arm.telescopic_servo(14)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(90)
    time.sleep(0.2)
    close_gripper_apple(arm)

    time.sleep(0.2)

    arm.telescopic_servo(0)
    arm.nod_servo(0)


def put_basket_to_robot(arm: 'ArmService', num):
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.gripper_servo(16)

    arm.lift(0, is_block=False)
    if num == 1:
        arm.rotate(20)
        arm.telescopic_servo(5.5)
        arm.rotary_servo(69)
        arm.wait_finish()
        arm.lift(1)
        arm.nod_servo(90)
    elif num == 2:
        arm.rotate(0)
        arm.rotary_servo(-85)
        arm.telescopic_servo(2)
        arm.wait_finish()
        arm.lift(1)
        arm.nod_servo(95)
    elif num == 3:
        arm.rotate(-23)
        arm.rotary_servo(-65)
        arm.telescopic_servo(5)
        arm.wait_finish()
        arm.lift(1)
        arm.nod_servo(90)
    else:
        raise ValueError("不正确的框子位置")

    time.sleep(0.5)
    arm.lift(0)
    arm.lift(10)
    open_gripper(arm)
    arm.lift(0)


def identify_ground_fruit(arm: 'ArmService'):
    arm.nod_servo(0)
    arm.lift(0, is_block=False)
    time.sleep(1)
    arm.rotate(180, is_block=False)
    arm.rotary_servo(0)
    arm.telescopic_servo(3)
    open_gripper(arm)
    arm.wait_finish()
    arm.nod_servo(90)
    time.sleep(1)


def identify_tree_fruit(arm: 'ArmService', direction: Direction):
    arm.lift(0)
    arm.nod_servo(0)
    if direction == Direction.LEFT:
        arm.rotate(180)
        arm.rotary_servo(90)
    elif direction == Direction.RIGHT:
        arm.rotate(-180)
        arm.rotary_servo(-90)
    else:
        raise ValueError("不支持的方向")
    arm.nod_servo(180)
    arm.lift(22, is_block=False)
    arm.telescopic_servo(0)
    open_gripper(arm)
    arm.wait_finish()
    time.sleep(2)


def put_fruit_to_basket(arm: 'ArmService', num):
    arm.lift(0)
    robot_basket_top(arm, num)
    open_gripper(arm)


def put_basket_to_station(arm: 'ArmService', direction: Direction):
    station_basket_top(arm, direction)

    arm.lift(7)
    open_gripper(arm)
    arm.lift(0)


def grab_apple_on_tree(arm: 'ArmService', direction: Direction, telescopic: float, is_low: bool):
    # 确定旋转方向
    if direction == Direction.LEFT:
        rotate_angle = 90
    elif direction == Direction.RIGHT:
        rotate_angle = -90
    else:
        raise ValueError("不支持的方向")

    # 抬升高度
    lift_height = 30 if is_low else 21

    # 初始姿态
    arm.lift(0, is_block=False)
    arm.rotate(rotate_angle, 40, is_block=False)
    arm.telescopic_servo(0)
    open_half_gripper(arm)

    if telescopic < 0:
        arm.nod_servo(90)
        arm.rotary_servo(180)
        arm.nod_servo(180)
    else:
        arm.nod_servo(0)
        arm.rotary_servo(0)
    arm.wait_finish()
    arm.lift(lift_height)

    # 伸出夹爪准备夹取
    if telescopic < 0:
        arm.nod_servo(180)
    else:
        arm.nod_servo(0)
    arm.telescopic_servo(telescopic)
    close_gripper_apple(arm)
    time.sleep(0.2)

    # 收回机械臂
    if is_low:
        if telescopic < 0:
            arm.nod_servo(120)
        else:
            arm.nod_servo(60)
        arm.telescopic_servo(0)
        arm.lift(23)
        arm.nod_servo(90)
    else:
        arm.nod_servo(90)
        arm.telescopic_servo(0)

    arm.lift(0)


def grab_grape_on_wall(arm: 'ArmService', direction: Direction, low=False):
    if low:
        lift = 32
        rotate = 150
        nod_servo = 50
        rotary_servo = 60
    else:
        lift = 32
        rotate = 160
        nod_servo = 0
        rotary_servo = 70

    arm.lift(lift, is_block=False)
    arm.nod_servo(90)

    if direction == Direction.LEFT:
        arm.rotary_servo(-rotary_servo)
        arm.rotate(rotate)
    if direction == Direction.RIGHT:
        arm.rotary_servo(rotary_servo)
        arm.rotate(-rotate)

    arm.nod_servo(nod_servo)
    arm.wait_finish()
    time.sleep(0.2)
    close_gripper_grape(arm)
    arm.rotary_servo(0)
    arm.telescopic_servo(0)
