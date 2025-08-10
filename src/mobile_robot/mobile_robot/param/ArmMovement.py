import time

from ..popo.Direction import Direction
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..service.ArmService import ArmService


def open_gripper(arm: 'ArmService'):
    arm.gripper_servo(26)


def open_half_gripper(arm: 'ArmService'):
    arm.gripper_servo(14)


def close_gripper_apple(arm: 'ArmService'):
    arm.gripper_servo(4)
    time.sleep(0.2)


def close_gripper_grape(arm: 'ArmService'):
    arm.gripper_servo(2)
    time.sleep(0.2)


def close_gripper_basket(arm: 'ArmService'):
    arm.gripper_servo(16)
    time.sleep(0.2)


def robot_basket_top(arm: 'ArmService', num, easy_mode=False):
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    if num == 1:
        arm.rotate(20, is_block=not easy_mode)
        arm.telescopic_servo(5.5)
        arm.rotary_servo(69)
        arm.wait_finish()
        arm.nod_servo(90)
    elif num == 2:
        arm.rotate(0, is_block=not easy_mode)
        arm.rotary_servo(-95)
        arm.telescopic_servo(4)
        arm.wait_finish()
        arm.nod_servo(105)
    elif num == 3:
        arm.rotate(-20, is_block=not easy_mode)
        arm.rotary_servo(-75)
        arm.telescopic_servo(5)
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
    arm.rotary_servo(0)
    arm.nod_servo(90)
    arm.telescopic_servo(0)
    open_half_gripper(arm)
    arm.wait_finish()
    arm.lift(10)


def motion(arm: 'ArmService'):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(0)
    arm.telescopic_servo(0)
    open_half_gripper(arm)
    arm.wait_finish()
    arm.lift(10)


def motion_apple(arm: 'ArmService'):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.telescopic_servo(0)
    close_gripper_apple(arm)
    arm.wait_finish()
    arm.lift(20)


def motion_grape(arm: 'ArmService'):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.telescopic_servo(0)
    close_gripper_grape(arm)
    arm.wait_finish()
    arm.lift(20)


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
    arm.nod_servo(10)
    if direction == Direction.LEFT:
        arm.rotate(190)
        arm.rotary_servo(-104)
    elif direction == Direction.RIGHT:
        arm.rotate(-190)
        arm.rotary_servo(97)
    arm.telescopic_servo(0)
    open_half_gripper(arm)
    arm.lift(35)
    arm.telescopic_servo(10)


def grab_basket_from_robot(arm: 'ArmService', num: int):
    robot_basket_top(arm, num, True)

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

    robot_basket_top(arm, num)
    time.sleep(0.5)

    arm.lift(10)
    open_gripper(arm)
    arm.lift(0)


def identify_ground_fruit(arm: 'ArmService'):
    arm.nod_servo(0)
    arm.rotate(180)
    arm.lift(0, is_block=False)
    arm.nod_servo(90)
    arm.rotary_servo(0)
    arm.telescopic_servo(3)
    open_gripper(arm)
    arm.wait_finish()


def identify_tree_fruit(arm: 'ArmService', direction: Direction):
    arm.lift(0)
    arm.nod_servo(0)
    if direction == Direction.LEFT:
        arm.rotate(180)
        arm.rotary_servo(-90)
    elif direction == Direction.RIGHT:
        arm.rotate(-180)
        arm.rotary_servo(90)
    else:
        raise ValueError("不支持的方向")
    arm.lift(35, is_block=False)
    arm.telescopic_servo(0)
    open_gripper(arm)
    arm.wait_finish()
    time.sleep(2)


def put_fruit_to_basket(arm: 'ArmService', num):
    arm.lift(0)
    arm.nod_servo(0)
    robot_basket_top(arm, num, True)
    open_gripper(arm)


def put_basket_to_station(arm: 'ArmService', direction: Direction):
    station_basket_top(arm, direction)

    arm.lift(7)
    open_gripper(arm)
    arm.lift(0)


def grab_apple_on_tree(arm: 'ArmService', direction: Direction, telescopic, is_low: bool):
    if direction == Direction.LEFT:
        rotate = 90
    elif direction == Direction.RIGHT:
        rotate = -90
    else:
        raise ValueError("不支持的方向")

    if is_low:
        lift = 35
    else:
        lift = 23

    arm.telescopic_servo(0)
    arm.lift(0)
    arm.rotate(rotate)
    arm.rotary_servo(0)
    arm.nod_servo(90)
    if is_low:
        arm.lift(23)
        arm.nod_servo(60)
    arm.lift(lift)

    open_half_gripper(arm)
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.telescopic_servo(telescopic)
    close_gripper_apple(arm)
    time.sleep(0.2)
    if is_low:
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
