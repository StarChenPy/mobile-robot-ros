import time

from ..popo.Direction import Direction
from ..service.ArmService import ArmService


def open_gripper(arm: ArmService):
    arm.gripper_servo(26)


def open_half_gripper(arm: ArmService):
    arm.gripper_servo(14)


def close_gripper_apple(arm: ArmService):
    arm.gripper_servo(4)
    time.sleep(0.2)


def close_gripper_grape(arm: ArmService):
    arm.gripper_servo(2)
    time.sleep(0.2)


def close_gripper_basket(arm: ArmService):
    arm.gripper_servo(15.3)
    time.sleep(0.2)


def robot_basket_top(arm: ArmService, num):
    if num == 1:
        # 1号位置正上方
        arm.lift(0)
        arm.rotate(22, is_block=False)
        arm.wait_finish()
        arm.nod_servo(90)
        arm.rotary_servo(73)
        time.sleep(1)
        arm.telescopic_servo(6)
    elif num == 2:
        # 2号位置正上方
        arm.lift(0)
        arm.rotate(-5, is_block=False)
        arm.wait_finish()
        arm.nod_servo(90)
        arm.rotary_servo(95)
        time.sleep(1)
        arm.telescopic_servo(4)
    elif num == 3:
        # 3号位置正上方
        arm.lift(0)
        arm.rotate(-28, is_block=False)
        arm.wait_finish()
        arm.nod_servo(90)
        arm.rotary_servo(120)
        time.sleep(1)
        arm.telescopic_servo(7)
    else:
        raise ValueError("不正确的框子位置")


def station_basket_top(arm: ArmService, direction: Direction):
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
    arm.telescopic_servo(0)
    arm.wait_finish()


def motion(arm: ArmService):
    arm.lift(0)
    arm.rotate(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.telescopic_servo(0)
    open_half_gripper(arm)
    arm.wait_finish()
    arm.lift(10)


def top_180(arm: ArmService):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.wait_finish()


def identify_station_fruit_right(arm: ArmService):
    arm.lift(0, is_block=False)
    arm.rotate(-90, is_block=False)
    arm.telescopic_servo(0)
    arm.nod_servo(45)
    open_gripper(arm)
    arm.wait_finish()
    time.sleep(1)


def identify_station_fruit_front(arm: ArmService):
    arm.lift(0, is_block=False)
    arm.rotate(180, is_block=False)
    arm.telescopic_servo(0)
    arm.nod_servo(45)
    open_gripper(arm)
    arm.wait_finish()
    time.sleep(1)


def identify_grape(arm: ArmService, direction: Direction):
    arm.nod_servo(5)
    if direction == Direction.LEFT:
        arm.rotate(190,40)
        arm.lift(35, 40,False)
        arm.rotary_servo(-100)
    elif direction == Direction.RIGHT:
        arm.rotate(-190,40)
        arm.lift(35, 40,False)
        arm.rotary_servo(100)
    arm.telescopic_servo(10)
    open_half_gripper(arm)
    arm.wait_finish()


def grab_basket_from_robot(arm: ArmService, num: int):
    robot_basket_top(arm, num)

    open_gripper(arm)

    arm.lift(10)
    close_gripper_basket(arm)
    arm.lift(0)
    arm.telescopic_servo(17)
    arm.rotary_servo(0)
    arm.nod_servo(0)


def grab_center_fruit_from_station(arm: ArmService):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(0)
    arm.telescopic_servo(4)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(70)
    time.sleep(1)
    close_gripper_apple(arm)

    time.sleep(1)

    arm.nod_servo(0)
    time.sleep(1)


def grab_left_fruit_from_station(arm: ArmService):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(-40)
    arm.telescopic_servo(17)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(70)
    time.sleep(0.5)
    close_gripper_apple(arm)

    time.sleep(0.5)

    arm.nod_servo(0)


def grab_right_fruit_from_station(arm: ArmService):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(40)
    arm.telescopic_servo(17)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(70)
    time.sleep(0.5)
    close_gripper_apple(arm)

    time.sleep(0.5)

    arm.nod_servo(0)


def grab_basket_from_station(arm: ArmService, direction: Direction):
    station_basket_top(arm, direction)
    open_gripper(arm)
    arm.nod_servo(0)
    arm.lift(7)
    close_gripper_basket(arm)
    arm.lift(0, is_block=False)
    arm.rotate(0, is_block=False)
    arm.wait_finish()


def put_basket_to_robot_b(arm: ArmService, num):
    arm.telescopic_servo(17)
    robot_basket_top(arm, num)

    arm.lift(10)
    open_gripper(arm)
    arm.lift(0)


def put_basket_to_robot_d(arm: ArmService, num):
    arm.nod_servo(30)
    if num == 1:
        arm.rotate(-5, 20, False)
        arm.rotary_servo(100)
        arm.telescopic_servo(5)
        arm.wait_finish()
    elif num == 2:
        arm.rotate(-32, 20, False)
        arm.rotary_servo(125)
        arm.telescopic_servo(9)
        arm.wait_finish()
    elif num == 3:
        arm.rotate(-43, 20, False)
        arm.rotary_servo(136)
        arm.telescopic_servo(15)
        arm.wait_finish()
    else:
        raise Exception('无效框子编号')
    arm.lift(18.5, 40, True)
    open_gripper(arm)
    arm.lift(0, 40, True)

def identify_ground_fruit(arm: ArmService):
    arm.nod_servo(0)
    arm.rotate(180)
    arm.lift(0, is_block=False)
    arm.nod_servo(90)
    arm.rotary_servo(0)
    arm.telescopic_servo(3)
    open_gripper(arm)
    arm.wait_finish()


def identify_tree_fruit(arm: ArmService, direction: Direction):
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


def put_fruit_to_basket(arm: ArmService, num):
    arm.nod_servo(0)
    robot_basket_top(arm, num)
    open_gripper(arm)


def put_basket_to_station(arm: ArmService, direction: Direction):
    station_basket_top(arm, direction)

    arm.lift(7)
    open_gripper(arm)
    arm.lift(0)


def grab_apple_on_tree(arm: ArmService, direction: Direction, telescopic, is_low: bool):
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
    if is_low:
        arm.nod_servo(0)
        arm.rotary_servo(90)
    else:
        arm.rotary_servo(0)
        arm.nod_servo(90)
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


def grab_grape_on_wall(arm: ArmService, direction: Direction, low = False):
    if low:
        lift = 32
        rotate = 135
        nod_servo = 50
        rotary_servo = 48
    else:
        lift = 32
        rotate = 152
        nod_servo = 0
        rotary_servo = 65

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
