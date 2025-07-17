import time

from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.ArmMovement import ArmMovement
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService

LIFT_HEIGHT = 50
DEFAULT_HEIGHT = 22
DEFAULT_GRIPPER = 12


def create_movement(rotate=None, lift=None, rotary=None, nod=None, telescopic=None, gripper=None):
    """
    创建动作
    方便更换高度
    """
    movement = ArmMovement()
    if rotate is not None and lift is not None:
        lift = max(lift - (50 - LIFT_HEIGHT), 0)
        movement.motor = MotorMovement(rotate, lift)
    if rotary is not None and nod is not None and telescopic is not None and gripper is not None:
        movement.servo = ServoMotor(rotary, nod, telescopic, gripper)

    return movement


# 基础动作
MOVING = create_movement(0, DEFAULT_HEIGHT, 0, 0, 0, 12)
MOVING_BANANA = create_movement(0, DEFAULT_HEIGHT, 0, 0, 0, 1.5)
MOVING_GRAPE = create_movement(0, DEFAULT_HEIGHT, 0, 0, 0, 2)


def recognition_orchard_apple_tree_1(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(90, DEFAULT_HEIGHT))
    arm.control(create_movement(90, 45, -90, 0, 0, 12))
    time.sleep(1)


def end_recognition_orchard_apple_tree_1(arm: ArmService):
    arm.control(create_movement(90, 1, -90, 0, 0, 12))
    arm.control(MOVING)


def recognition_orchard_apple_tree_2(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(180, DEFAULT_HEIGHT))
    arm.control(create_movement(180, 45, 0, 0, 0, 24))
    time.sleep(1)


def end_recognition_orchard_apple_tree_2(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(180, 1, 0, 0, 0, 24))
    arm.control(MOVING)
    time.sleep(1)


def recognition_orchard_banana_tree(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(180, DEFAULT_HEIGHT, 0, 0, 0, 24))
    time.sleep(1)


def end_recognition_orchard_banana_tree(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(180, DEFAULT_HEIGHT, 0, 0, 0, 24))
    arm.control(MOVING)
    time.sleep(1)


def recognition_orchard_grape(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(90, DEFAULT_HEIGHT))
    arm.control(create_movement(90, 45, -90, 0, 0, 24))
    time.sleep(1)


def end_recognition_orchard_grape(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(create_movement(90, 1, -90, 0, 0, 24))
    arm.control(MOVING)
    time.sleep(1)


def put_fruit_into_basket(arm: ArmService, box_number: int, gripper: float) -> None:
    """将水果放置到指定框子中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 28
        telescopic = 1.5
    elif box_number == 2:
        arm_pos = 0
        telescopic = 0
    elif box_number == 3:
        arm_pos = -30
        telescopic = 1.5
    else:
        raise ValueError("篮子编号无效")

    arm.control(create_movement(arm_pos, 4, 0, 90, telescopic, gripper))
    arm.control(create_movement(arm_pos, 4, 0, 90, telescopic, 12))
    time.sleep(0.5)


def put_basket_to_robot(arm: ArmService, box_number: int) -> None:
    """将框子放到机器人中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 22
        servo_params = (-72, 5)
    elif box_number == 2:
        arm_pos = 10
        servo_params = (100, 3.5)
    elif box_number == 3:
        arm_pos = -20
        servo_params = (68, 5)
    else:
        raise ValueError("篮子编号无效")

    rotary, telescopic = servo_params

    arm.control(create_movement(arm_pos, 1, rotary, 0, telescopic, 15))
    arm.control(create_movement(arm_pos, 1, rotary, 90, telescopic, 15))
    time.sleep(0.5)
    arm.control(create_movement(arm_pos, 15))
    arm.control(create_movement(arm_pos, 15, rotary, 90, telescopic, 24))
    time.sleep(0.5)
    arm.control(create_movement(arm_pos, 1))
    arm.control(create_movement(0, 1, 0, 0, 3, 6.5))


def grab_basket_from_station(arm: ArmService) -> None:
    arm.control(create_movement(180, 1, 0, 0, 6, 24))
    arm.control(create_movement(180, 16, 0, 0, 6, 24))
    arm.control(create_movement(180, 16, 0, 0, 6, 15))
    arm.control(create_movement(180, 1, 0, 0, 0, 15))
    arm.control(create_movement(0, 1))


def grab_basket_to_station(arm: ArmService, box_number: int) -> None:
    """将指定框子放到面前的站台上"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 22
        servo_params = (-72, 5)
    elif box_number == 2:
        arm_pos = 10
        servo_params = (100, 3.5)
    elif box_number == 3:
        arm_pos = -20
        servo_params = (68, 5)
    else:
        raise ValueError("篮子编号无效")

    rotary, telescopic = servo_params

    # 准备抓
    arm.control(create_movement(0, 1, 0, 0, 3, 6.5))
    arm.control(create_movement(arm_pos, 1, rotary, 90, telescopic, 24))
    time.sleep(0.5)
    arm.control(create_movement(arm_pos, 16, rotary, 90, telescopic, 24))
    # 夹合
    arm.control(create_movement(arm_pos, 16, rotary, 90, telescopic, 15))
    time.sleep(0.5)
    # 提起
    arm.control(create_movement(arm_pos, 1, rotary, 90, telescopic, 15))
    arm.control(create_movement(arm_pos, 1, 0, 90, telescopic, 15))
    time.sleep(0.5)
    arm.control(create_movement(arm_pos, 1, 0, 0, 5, 15))

    # 放下（公共部分）
    arm.control(create_movement(180, 1, 0, 0, 10, 15))
    arm.control(create_movement(180, 1, 0, 0, 10, 24))
    arm.control(create_movement(180, 1, 0, 90, 0, 24))
    time.sleep(0.5)
    # 结束（公共部分）
    arm.control(create_movement(0, 1))
    arm.control(create_movement(0, 1, 0, 0, 0, 6.5))


def grab_basket_to_warehouse(arm: ArmService, box_number: int) -> None:
    """将指定框子放到面前的仓库中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 22
        servo_params = (-72, 5)
    elif box_number == 2:
        arm_pos = 10
        servo_params = (98, 3.5)
    elif box_number == 3:
        arm_pos = -20
        servo_params = (68, 5)
    else:
        raise ValueError("篮子编号无效")

    rotary, telescopic = servo_params

    arm.control(create_movement(arm_pos, 1, rotary, 0, telescopic, 24))
    arm.control(create_movement(arm_pos, 1, rotary, 90, telescopic, 24))
    time.sleep(0.5)
    arm.control(create_movement(arm_pos, 15))
    arm.control(create_movement(arm_pos, 15, rotary, 90, telescopic, 15))
    time.sleep(0.5)
    arm.control(create_movement(arm_pos, 1))

    # 放下（公共部分）
    arm.control(create_movement(180, 1,  0, 90, 10, 15))
    arm.control(create_movement(180, 30))
    arm.control(create_movement(None, None, 0, 90, 10, 24))
    arm.control(create_movement(None, None, 0, 0, 0, 24))
    # 结束（公共部分）
    arm.control(MOVING)


def grab_apple_on_tree(arm, location_on_tree: FruitLocationOnTree):
    match location_on_tree:
        case FruitLocationOnTree.TOP_CENTER:
            arm.control(create_movement(180, 1, 0, 58, 0, 12))
            arm.control(create_movement(180, 22))
            arm.control(create_movement(None, None, 0, 58, 0, 4))
            arm.control(create_movement(None, None, 0, 90, 0, 4))
            arm.control(create_movement(180, 1))
        case FruitLocationOnTree.BOTTOM_CENTER:
            arm.control(create_movement(180, 1, 90, 30, 0, 12))
            arm.control(create_movement(180, 50))
            arm.control(create_movement(None, None, 0, 30, 0, 12))
            arm.control(create_movement(None, None, 0, -15, 1, 12))
            arm.control(create_movement(None, None, 0, -15, 1, 4))
            arm.control(create_movement(None, None, 90, -15, 0, 4))
            arm.control(create_movement(180, 1))
        case FruitLocationOnTree.TOP_LEFT:
            arm.control(create_movement(155, 1, -10, 0, 0, 12))
            arm.control(create_movement(155, 37))
            arm.control(create_movement(None, None, -10, 0, 13, 12))
            arm.control(create_movement(None, None, -10, 0, 13, 4))
            arm.control(create_movement(None, None, 30, 0, 0, 4))
            arm.control(create_movement(155, 1))
        case FruitLocationOnTree.BOTTOM_LEFT:
            arm.control(create_movement(151, 1, 0, 0, 0, 12))
            arm.control(create_movement(151, 50))
            arm.control(create_movement(None, None, -40, 20, 0, 12))
            arm.control(create_movement(None, None, -68, 20, 18, 12))
            arm.control(create_movement(None, None, -68, -10, 18, 12))
            arm.control(create_movement(None, None, -68, -10, 18, 4))
            arm.control(create_movement(None, None, -40, 0, 17, 4))
            arm.control(create_movement(None, None, -40, 0, 0, 4))
            arm.control(create_movement(None, None, 20, 0, 0, 4))
            arm.control(create_movement(151, 1))
        case FruitLocationOnTree.TOP_RIGHT:
            arm.control(create_movement(-155, 1, 10, 0, 0, 12))
            arm.control(create_movement(-155, 37))
            arm.control(create_movement(None, None, 10, 0, 13, 12))
            arm.control(create_movement(None, None, 10, 0, 13, 4))
            arm.control(create_movement(None, None, -30, 0, 13, 4))
            arm.control(create_movement(None, None, -30, 0, 0, 4))
            arm.control(create_movement(-155, 1))
        case FruitLocationOnTree.BOTTOM_RIGHT:
            arm.control(create_movement(-151, 1, 0, 0, 0, 12))
            arm.control(create_movement(-151, 50))
            arm.control(create_movement(None, None, 40, 20, 0, 12))
            arm.control(create_movement(None, None, 68, 20, 18, 12))
            arm.control(create_movement(None, None, 68, -10, 18, 12))
            arm.control(create_movement(None, None, 68, -10, 18, 4))
            arm.control(create_movement(None, None, -20, -10, 17, 4))
            arm.control(create_movement(None, None, -20, 0, 0, 4))
            arm.control(create_movement(-151, 1))


def grab_banana_on_tree(arm, location_on_tree: FruitLocationOnTree):
    match location_on_tree:
        case FruitLocationOnTree.TOP_CENTER:
            arm.control(create_movement(180, 1, 0, 90, 0, 12))
            arm.control(create_movement(None, None, 0, 35, 0, 12))
            arm.control(create_movement(None, None, 0, 35, 0, 1.5))
            arm.control(create_movement(0, 1, 0, 90, 0, 1.5))
        case FruitLocationOnTree.BOTTOM_CENTER:
            arm.control(create_movement(155, 1, 0, 90, 0, 12))
            arm.control(create_movement(180, 14))
            arm.control(create_movement(None, None, 0, 70, 0, 12))
            arm.control(create_movement(None, None, 0, 70, 0, 1.5))
            arm.control(create_movement(100, 14))
            # arm.control(create_movement(0, 1, 0, 0, 0, 1.5))
        case FruitLocationOnTree.TOP_LEFT:
            arm.control(create_movement(155, 10, -30, 0, 0, 12))
            arm.control(create_movement(None, None, -30, 0, 14, 12))
            arm.control(create_movement(None, None, -30, 0, 14, 1.5))
        case FruitLocationOnTree.BOTTOM_LEFT:
            arm.control(create_movement(155, 1, -10, 0, 0, 12))
            arm.control(create_movement(155, 30))
            arm.control(create_movement(None, None, -10, 0, 14, 12))
            arm.control(create_movement(None, None, -10, 0, 14, 1.5))
            arm.control(create_movement(None, None, 50, 0, 14, 1.5))
            arm.control(create_movement(None, None, 50, 0, 0, 1.5))
            arm.control(create_movement(155, 1))
        case FruitLocationOnTree.TOP_RIGHT:
            arm.control(create_movement(-151, 10, 40, 0, 0, 12))
            arm.control(create_movement(None, None, 40, 0, 14, 12))
            arm.control(create_movement(None, None, 40, 0, 14, 1.5))
        case FruitLocationOnTree.BOTTOM_RIGHT:
            arm.control(create_movement(-155, 1, 10, 0, 0, 12))
            arm.control(create_movement(-155, 30))
            arm.control(create_movement(None, None, 10, 0, 14, 12))
            arm.control(create_movement(None, None, 10, 0, 14, 1.5))
            arm.control(create_movement(None, None, -50, 0, 14, 1.5))
            arm.control(create_movement(None, None, -50, 0, 0, 1.5))
            arm.control(create_movement(-155, 1))


def put_fruit_into_warehouse(arm, gripper):
    arm.control(create_movement(180, 20, 0, 0, 0, gripper))
    arm.control(create_movement(180, 43, 0, 90, 10, gripper))
    arm.control(create_movement(None, None, 0, 90, 10, 12))
    arm.control(create_movement(180, 20))
    arm.control(MOVING)


def grab_grape(arm):
    arm.control(create_movement(180, 1, 90, 0, 0, 24))
    arm.control(create_movement(180, 45, 70, 0, 8, 24))
    arm.control(create_movement(161, 45))
    arm.control(create_movement(None, None, 70, 0, 8, 2))
    arm.control(create_movement(180, 1))
