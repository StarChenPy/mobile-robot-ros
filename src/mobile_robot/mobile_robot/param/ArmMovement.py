import time

from ..popo.Direction import Direction
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..service.ArmService import ArmService


def open_gripper(arm: ArmService):
    arm.gripper_servo(24)


def open_half_gripper(arm: ArmService):
    arm.gripper_servo(12)


def close_gripper(arm: ArmService, size):
    arm.gripper_servo(size)
    time.sleep(0.2)


def robot_basket_top(arm: ArmService, num):
    if num == 1:
        # 1号位置正上方
        arm.lift(0, is_block=False)
        arm.rotate(22, is_block=False)
        arm.wait_finish()
        arm.telescopic_servo(17)
        arm.nod_servo(90)
        arm.rotary_servo(73)
        arm.telescopic_servo(7)
    elif num == 2:
        # 2号位置正上方
        arm.lift(0, is_block=False)
        arm.rotate(-5, is_block=False)
        arm.wait_finish()
        arm.telescopic_servo(17)
        arm.nod_servo(90)
        arm.rotary_servo(95)
        arm.telescopic_servo(4)
    elif num == 3:
        # 3号位置正上方
        arm.lift(0, is_block=False)
        arm.rotate(-28, is_block=False)
        arm.wait_finish()
        arm.telescopic_servo(17)
        arm.nod_servo(90)
        arm.rotary_servo(120)
        arm.telescopic_servo(7)
    else:
        raise ValueError("不正确的框子位置")


def station_basket_top(arm: ArmService, direction: Direction):
    if direction == Direction.FRONT:
        rotate = 180
    elif direction == Direction.LEFT:
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
    arm.rotate(0, is_block=False)
    arm.lift(0, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.telescopic_servo(0)
    open_gripper(arm)
    arm.wait_finish()
    arm.lift(10)


def top_180(arm: ArmService):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.wait_finish()


def grab_motion(arm: ArmService, size):
    arm.rotate(0, is_block=False)
    arm.lift(10, is_block=False)
    arm.rotary_servo(0)
    arm.nod_servo(0)
    arm.telescopic_servo(0)
    close_gripper(arm, size)
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
    if direction == Direction.LEFT:
        arm.rotate(190,40)
        arm.lift(35, 40,False)
        arm.nod_servo(5)
        arm.rotary_servo(-100)
        arm.telescopic_servo(10)
        open_half_gripper(arm)
        arm.wait_finish()


def grab_basket_from_robot(arm: ArmService, num: int):
    robot_basket_top(arm, num)

    open_gripper(arm)

    arm.lift(10)
    close_gripper(arm, 15)
    arm.lift(0)
    arm.telescopic_servo(17)
    arm.rotary_servo(0)
    arm.nod_servo(0)


def grab_center_fruit_from_station(arm: ArmService, fruit_size):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(0)
    arm.telescopic_servo(4)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(60)
    time.sleep(1)
    close_gripper(arm, fruit_size)

    time.sleep(1)

    arm.nod_servo(0)
    time.sleep(1)


def grab_left_fruit_from_station(arm: ArmService, fruit_size):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(40)
    arm.telescopic_servo(18)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(60)
    time.sleep(0.5)
    close_gripper(arm, fruit_size)

    time.sleep(0.5)

    arm.nod_servo(0)


def grab_right_fruit_from_station(arm: ArmService, fruit_size):
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(-40)
    arm.telescopic_servo(18)
    open_gripper(arm)
    arm.wait_finish()

    arm.nod_servo(60)
    time.sleep(0.5)
    close_gripper(arm, fruit_size)

    time.sleep(0.5)

    arm.nod_servo(0)


def grab_basket_from_station(arm: ArmService, direction: Direction):
    station_basket_top(arm, direction)
    open_gripper(arm)
    arm.lift(10)
    close_gripper(arm, 15)
    arm.lift(0, is_block=False)
    arm.rotate(0, is_block=False)
    arm.wait_finish()


def put_basket_to_robot_b(arm: ArmService, num):
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
    arm.rotate(180, is_block=False)
    arm.lift(0, is_block=False)
    arm.nod_servo(90)
    arm.rotary_servo(0)
    arm.telescopic_servo(3)
    open_gripper(arm)
    arm.wait_finish()


def identify_tree_fruit(arm: ArmService):
    arm.rotate(180, is_block=False)
    arm.lift(35, is_block=False)
    arm.nod_servo(0)
    arm.rotary_servo(0)
    arm.telescopic_servo(0)
    open_gripper(arm)
    arm.wait_finish()
    time.sleep(2)


def put_fruit_to_basket(arm: ArmService, num):
    robot_basket_top(arm, num)
    open_gripper(arm)


def put_basket_to_station(arm: ArmService, direction: Direction):
    station_basket_top(arm, direction)

    arm.lift(10)
    open_gripper(arm)
    arm.lift(0)


def grab_apple_on_tree(arm: ArmService, location_on_tree: FruitLocationOnTree):
    FRUIT_SIZE = 4

    match location_on_tree:
        case FruitLocationOnTree.TOP_CENTER:
            # 准备姿态
            arm.rotate(180, is_block=False)
            arm.lift(0, is_block=False)
            arm.nod_servo(0)
            open_gripper(arm)
            arm.rotary_servo(0)
            arm.telescopic_servo(0)
            arm.nod_servo(58)
            arm.wait_finish()

            # 下降
            arm.lift(10, 40)

            # 抓取与返回
            close_gripper(arm, FRUIT_SIZE)
            time.sleep(0.5)
            arm.nod_servo(90)
            arm.lift(0, 40)
            arm.rotate(150, 40)
        case FruitLocationOnTree.BOTTOM_CENTER:
            # 准备姿态
            arm.rotate(180, is_block=False)
            arm.lift(0, is_block=False)
            arm.rotary_servo(90)
            arm.nod_servo(30)
            arm.telescopic_servo(0)
            open_half_gripper(arm)
            arm.wait_finish()

            # 下降
            arm.lift(35, 40)

            # 抓取与返回
            arm.rotary_servo(0)
            arm.nod_servo(-15)
            arm.telescopic_servo(1)
            time.sleep(0.5)
            arm.gripper_servo(FRUIT_SIZE)
            arm.nod_servo(0)
            arm.rotary_servo(90)
            arm.lift(0, 40)
        case FruitLocationOnTree.TOP_LEFT:
            arm.rotate(150, is_block=False)
            arm.lift(0, is_block=False)
            arm.rotary_servo(20)
            arm.nod_servo(0)
            arm.telescopic_servo(0)
            open_half_gripper(arm)
            arm.wait_finish()

            arm.lift(27, 40)

            arm.nod_servo(-10)
            arm.telescopic_servo(13)
            time.sleep(0.5)
            close_gripper(arm, FRUIT_SIZE)
            arm.nod_servo(30)
            arm.telescopic_servo(0)
            arm.lift(0, 40)
        case FruitLocationOnTree.BOTTOM_LEFT:
            arm.rotate(151, is_block=False)
            arm.lift(0, is_block=False)
            arm.rotary_servo(0)
            arm.nod_servo(0)
            arm.telescopic_servo(0)
            open_half_gripper(arm)
            arm.wait_finish()

            arm.lift(35, 40)

            arm.rotary_servo(40)
            arm.nod_servo(20)
            arm.telescopic_servo(17)
            arm.rotary_servo(68)
            time.sleep(0.5)
            arm.nod_servo(-10)
            close_gripper(arm, FRUIT_SIZE)
            arm.rotary_servo(40)
            arm.telescopic_servo(0)

            arm.rotary_servo(-20)
            arm.lift(0, 40)
        case FruitLocationOnTree.TOP_RIGHT:
            arm.rotate(-150, is_block=False)
            arm.lift(0, is_block=False)
            arm.rotary_servo(-20)
            arm.nod_servo(0)
            arm.telescopic_servo(0)
            open_half_gripper(arm)
            arm.wait_finish()

            arm.lift(27, 40)

            arm.nod_servo(-10)
            arm.telescopic_servo(13)
            time.sleep(0.5)
            close_gripper(arm, FRUIT_SIZE)
            arm.nod_servo(30)
            arm.rotary_servo(60)
            arm.telescopic_servo(0)
            arm.lift(0, 40)
        case FruitLocationOnTree.BOTTOM_RIGHT:
            arm.rotate(-151, is_block=False)
            arm.lift(0, is_block=False)
            arm.rotary_servo(0)
            arm.nod_servo(0)
            arm.telescopic_servo(0)
            open_half_gripper(arm)
            arm.wait_finish()

            arm.lift(35, 40)

            arm.rotary_servo(-40)
            arm.nod_servo(20)
            arm.telescopic_servo(17)
            arm.rotary_servo(-68)
            time.sleep(0.5)
            arm.nod_servo(-10)
            close_gripper(arm, FRUIT_SIZE)
            arm.rotary_servo(-40)
            arm.telescopic_servo(0)

            arm.rotary_servo(20)
            arm.lift(0, 40)


def grab_grape_on_wall(arm: ArmService, direction: Direction, low = False):
    if direction == Direction.LEFT:
        if low:
            arm.lift(32, is_block=False)
            arm.rotate(150, is_block=False)
            arm.nod_servo(50)
            arm.rotary_servo(-60)
            time.sleep(0.2)
            close_gripper(arm, 2)
            arm.wait_finish()
        else:
            arm.lift(30, is_block=False)
            arm.rotate(160, is_block=False)
            arm.rotary_servo(-70)
            time.sleep(0.2)
            close_gripper(arm, 2)
            arm.wait_finish()
