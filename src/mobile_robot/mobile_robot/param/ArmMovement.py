import time

from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.ArmMovement import ArmMovement
from ..popo.Direction import Direction
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService

# 基础动作
MOVING = ArmMovement(MotorMovement(0, 20), ServoMotor(0, 0, 0, 4))
MOVING_BANANA = ArmMovement(MotorMovement(0, 20), ServoMotor(0, 0, 0, 1.5))
MOVING_GRAPE = ArmMovement(MotorMovement(0, 20), ServoMotor(0, 0, 0, 2))

# 识别果仓中的水果动作
RECOGNITION_WAREHOUSE = ArmMovement(MotorMovement(175, 15), ServoMotor(0, -90, 14, 20))

# 放水果到果仓
READY_PULL_WAREHOUSE = ArmMovement(MotorMovement(180, 16), ServoMotor(0, 0, 8, 6.5))
PULL_WAREHOUSE = ArmMovement(servo=ServoMotor(0, 0, 8, 16))


def recognition_orchard_apple_tree(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(ArmMovement(MotorMovement(90, 20)))
    arm.control(ArmMovement(MotorMovement(90, 45), ServoMotor(-90, 0, 0, 12)))
    time.sleep(1)


def recognition_orchard_banana_tree(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(ArmMovement(MotorMovement(180, 22), ServoMotor(0, 0, 0, 24)))
    time.sleep(1)


def recognition_orchard_grape(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(ArmMovement(MotorMovement(90, 20)))
    arm.control(ArmMovement(MotorMovement(90, 45), ServoMotor(-90, 0, 0, 24)))
    time.sleep(1)


def recognition_orchard(arm: ArmService, direction: Direction.LEFT or Direction.RIGHT):
    """调整为识别姿态"""

    if direction == Direction.LEFT:
        arm_pos = 90
    elif direction == Direction.RIGHT:
        arm_pos = -90
    else:
        raise ValueError("不可用的Direction")

    arm.control(ArmMovement(MotorMovement(arm_pos, 6), ServoMotor(0, -90, 15, 20)))


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

    arm.control(ArmMovement(MotorMovement(arm_pos, 8), ServoMotor(0, 90, telescopic, gripper)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 8), ServoMotor(0, 90, telescopic, 12)))
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

    arm.control(ArmMovement(MotorMovement(arm_pos, 1), ServoMotor(rotary, 0, telescopic, 15)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 1), ServoMotor(rotary, 90, telescopic, 15)))
    time.sleep(0.5)
    arm.control(ArmMovement(MotorMovement(arm_pos, 15)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 15), ServoMotor(rotary, 90, telescopic, 24)))
    time.sleep(0.5)
    arm.control(ArmMovement(MotorMovement(arm_pos, 1)))
    arm.control(ArmMovement(MotorMovement(0, 1), ServoMotor(0, 0, 3, 6.5)))


def grab_basket_from_station(arm: ArmService) -> None:
    arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 0, 6, 24)))
    arm.control(ArmMovement(MotorMovement(180, 16), ServoMotor(0, 0, 6, 24)))
    arm.control(ArmMovement(MotorMovement(180, 16), ServoMotor(0, 0, 6, 15)))
    arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 0, 0, 15)))
    arm.control(ArmMovement(MotorMovement(0, 1)))


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
    arm.control(ArmMovement(MotorMovement(0, 1), ServoMotor(0, 0, 3, 6.5)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 1), ServoMotor(rotary, 90, telescopic, 24)))
    time.sleep(0.5)
    arm.control(ArmMovement(MotorMovement(arm_pos, 16), ServoMotor(rotary, 90, telescopic, 24)))
    # 夹合
    arm.control(ArmMovement(MotorMovement(arm_pos, 16), ServoMotor(rotary, 90, telescopic, 15)))
    time.sleep(0.5)
    # 提起
    arm.control(ArmMovement(MotorMovement(arm_pos, 1), ServoMotor(rotary, 90, telescopic, 15)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 1), ServoMotor(0, 90, telescopic, 15)))
    time.sleep(0.5)
    arm.control(ArmMovement(MotorMovement(arm_pos, 1), ServoMotor(0, 0, 5, 15)))

    # 放下（公共部分）
    arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 0, 10, 15)))
    arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 0, 10, 24)))
    arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 90, 0, 24)))
    time.sleep(0.5)
    # 结束（公共部分）
    arm.control(ArmMovement(MotorMovement(0, 1)))
    arm.control(ArmMovement(MotorMovement(0, 1), ServoMotor(0, 0, 0, 6.5)))


def grab_basket_to_warehouse(arm: ArmService, box_number: int) -> None:
    """将指定框子放到面前的仓库中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 30
        servo_params = (-56, -90, 15)
    elif box_number == 2:
        arm_pos = 0
        servo_params = (90, -90, 12)
    elif box_number == 3:
        arm_pos = -30
        servo_params = (60, -90, 15)
    else:
        raise ValueError("篮子编号无效")

    rotary, nod, telescopic = servo_params

    # 准备抓
    arm.control(ArmMovement(MotorMovement(0, 10), ServoMotor(0, 0, 3, 6.5)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 6), ServoMotor(rotary, nod, telescopic, 25)))
    time.sleep(0.5)
    arm.control(ArmMovement(MotorMovement(arm_pos, 12), ServoMotor(rotary, nod, telescopic, 25)))
    # 夹合
    arm.control(ArmMovement(MotorMovement(arm_pos, 12), ServoMotor(rotary, nod, telescopic, 19.5)))
    time.sleep(0.5)
    # 提起
    arm.control(ArmMovement(MotorMovement(arm_pos, 2), ServoMotor(rotary, nod, telescopic, 19.5)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 2), ServoMotor(0, nod, telescopic, 19.5)))
    time.sleep(0.5)
    arm.control(ArmMovement(MotorMovement(arm_pos, 2), ServoMotor(0, 0, 5, 18.5)))

    # 放下（公共部分）
    arm.control(ArmMovement(MotorMovement(180, 2), ServoMotor(0, 0, 5, 18.5)))
    arm.control(ArmMovement(MotorMovement(180, 25), ServoMotor(0, 0, 5, 18.5)))
    arm.control(ArmMovement(MotorMovement(180, 25), ServoMotor(0, 0, 5, 25)))
    time.sleep(0.5)
    # 结束（公共部分）
    arm.control(ArmMovement(MotorMovement(0, 16), ServoMotor(0, 0, 3, 6.5)))


def grab_apple_on_tree(arm, location_on_tree: FruitLocationOnTree):
    match location_on_tree:
        case FruitLocationOnTree.TOP_CENTER:
            arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 58, 0, 12)))
            arm.control(ArmMovement(MotorMovement(180, 22)))
            arm.control(ArmMovement(servo=ServoMotor(0, 58, 0, 4)))
            arm.control(ArmMovement(servo=ServoMotor(0, 90, 0, 4)))
            arm.control(ArmMovement(MotorMovement(180, 1)))
        case FruitLocationOnTree.BOTTOM_CENTER:
            arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(90, 30, 0, 12)))
            arm.control(ArmMovement(MotorMovement(180, 50)))
            arm.control(ArmMovement(servo=ServoMotor(0, 30, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(0, -15, 1, 12)))
            arm.control(ArmMovement(servo=ServoMotor(0, -15, 1, 4)))
            arm.control(ArmMovement(servo=ServoMotor(90, -15, 0, 4)))
            arm.control(ArmMovement(MotorMovement(180, 1)))
        case FruitLocationOnTree.TOP_LEFT:
            arm.control(ArmMovement(MotorMovement(155, 1), ServoMotor(-10, 0, 0, 12)))
            arm.control(ArmMovement(MotorMovement(155, 37)))
            arm.control(ArmMovement(servo=ServoMotor(-10, 0, 13, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-10, 0, 13, 4)))
            arm.control(ArmMovement(servo=ServoMotor(30, 0, 0, 4)))
            arm.control(ArmMovement(MotorMovement(155, 1)))
        case FruitLocationOnTree.BOTTOM_LEFT:
            arm.control(ArmMovement(MotorMovement(151, 1), ServoMotor(0, 0, 0, 12)))
            arm.control(ArmMovement(MotorMovement(151, 50)))
            arm.control(ArmMovement(servo=ServoMotor(-40, 20, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-68, 20, 18, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-68, -10, 18, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-68, -10, 18, 4)))
            arm.control(ArmMovement(servo=ServoMotor(-40, 0, 17, 4)))
            arm.control(ArmMovement(servo=ServoMotor(-40, 0, 0, 4)))
            arm.control(ArmMovement(servo=ServoMotor(20, 0, 0, 4)))
            arm.control(ArmMovement(MotorMovement(151, 1)))
        case FruitLocationOnTree.TOP_RIGHT:
            arm.control(ArmMovement(MotorMovement(-155, 1), ServoMotor(10, 0, 0, 12)))
            arm.control(ArmMovement(MotorMovement(-155, 37)))
            arm.control(ArmMovement(servo=ServoMotor(10, 0, 13, 12)))
            arm.control(ArmMovement(servo=ServoMotor(10, 0, 13, 4)))
            arm.control(ArmMovement(servo=ServoMotor(-30, 0, 13, 4)))
            arm.control(ArmMovement(servo=ServoMotor(-30, 0, 0, 4)))
            arm.control(ArmMovement(MotorMovement(-155, 1)))
        case FruitLocationOnTree.BOTTOM_RIGHT:
            arm.control(ArmMovement(MotorMovement(-151, 1), ServoMotor(0, 0, 0, 12)))
            arm.control(ArmMovement(MotorMovement(-151, 50)))
            arm.control(ArmMovement(servo=ServoMotor(40, 20, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(68, 20, 18, 12)))
            arm.control(ArmMovement(servo=ServoMotor(68, -10, 18, 12)))
            arm.control(ArmMovement(servo=ServoMotor(68, -10, 18, 4)))
            arm.control(ArmMovement(servo=ServoMotor(-20, -10, 17, 4)))
            arm.control(ArmMovement(servo=ServoMotor(-20, 0, 0, 4)))
            arm.control(ArmMovement(MotorMovement(-151, 1)))


def grab_banana_on_tree(arm, location_on_tree: FruitLocationOnTree):
    match location_on_tree:
        case FruitLocationOnTree.TOP_CENTER:
            arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(0, 90, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(0, 35, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(0, 35, 0, 1.5)))
            arm.control(ArmMovement(MotorMovement(0, 1), ServoMotor(0, 90, 0, 1.5)))
        case FruitLocationOnTree.BOTTOM_CENTER:
            arm.control(ArmMovement(MotorMovement(155, 1), ServoMotor(0, 90, 0, 12)))
            arm.control(ArmMovement(MotorMovement(180, 14)))
            arm.control(ArmMovement(servo=ServoMotor(0, 70, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(0, 70, 0, 1.5)))
            arm.control(ArmMovement(MotorMovement(100, 14)))
            # arm.control(ArmMovement(MotorMovement(0, 1), ServoMotor(0, 0, 0, 1.5)))
        case FruitLocationOnTree.TOP_LEFT:
            arm.control(ArmMovement(MotorMovement(155, 10), ServoMotor(-30, 0, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-30, 0, 14, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-30, 0, 14, 1.5)))
        case FruitLocationOnTree.BOTTOM_LEFT:
            arm.control(ArmMovement(MotorMovement(155, 1), ServoMotor(-10, 0, 0, 12)))
            arm.control(ArmMovement(MotorMovement(155, 30)))
            arm.control(ArmMovement(servo=ServoMotor(-10, 0, 14, 12)))
            arm.control(ArmMovement(servo=ServoMotor(-10, 0, 14, 1.5)))
            arm.control(ArmMovement(servo=ServoMotor(50, 0, 14, 1.5)))
            arm.control(ArmMovement(servo=ServoMotor(50, 0, 0, 1.5)))
            arm.control(ArmMovement(MotorMovement(155, 1)))
        case FruitLocationOnTree.TOP_RIGHT:
            arm.control(ArmMovement(MotorMovement(-151, 10), ServoMotor(40, 0, 0, 12)))
            arm.control(ArmMovement(servo=ServoMotor(40, 0, 14, 12)))
            arm.control(ArmMovement(servo=ServoMotor(40, 0, 14, 1.5)))
        case FruitLocationOnTree.BOTTOM_RIGHT:
            arm.control(ArmMovement(MotorMovement(-155, 1), ServoMotor(10, 0, 0, 12)))
            arm.control(ArmMovement(MotorMovement(-155, 30)))
            arm.control(ArmMovement(servo=ServoMotor(10, 0, 14, 12)))
            arm.control(ArmMovement(servo=ServoMotor(10, 0, 14, 1.5)))
            arm.control(ArmMovement(servo=ServoMotor(-50, 0, 14, 1.5)))
            arm.control(ArmMovement(servo=ServoMotor(-50, 0, 0, 1.5)))
            arm.control(ArmMovement(MotorMovement(-155, 1)))


def put_fruit_into_warehouse(arm, gripper):
    arm.control(ArmMovement(MotorMovement(180, 20), ServoMotor(0, 0, 0, gripper)))
    arm.control(ArmMovement(MotorMovement(180, 43), ServoMotor(0, 90, 10, gripper)))
    arm.control(ArmMovement(servo=ServoMotor(0, 90, 10, 12)))
    arm.control(ArmMovement(MotorMovement(180, 20)))
    arm.control(MOVING)


def grab_grape(arm):
    arm.control(ArmMovement(MotorMovement(180, 1), ServoMotor(90, 0, 0, 24)))
    arm.control(ArmMovement(MotorMovement(180, 45), ServoMotor(70, 0, 8, 24)))
    arm.control(ArmMovement(MotorMovement(161, 45)), 20)
    arm.control(ArmMovement(servo=ServoMotor(70, 0, 8, 2)))
    arm.control(ArmMovement(MotorMovement(180, 1)))
