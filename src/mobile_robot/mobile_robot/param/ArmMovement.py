import time

from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.ArmMovement import ArmMovement
from ..popo.Direction import Direction
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService

# 基础动作
MOVING = ArmMovement(MotorMovement(0, 16), ServoMotor(0, 0, 3, 6.5))
TEST = ArmMovement(MotorMovement(90, 15))
ZERO = ArmMovement(MotorMovement(0, 15))

# 识别果仓中的水果动作
RECOGNITION_WAREHOUSE = ArmMovement(MotorMovement(175, 15), ServoMotor(0, -90, 14, 20))

# 放水果到果仓
READY_PULL_WAREHOUSE = ArmMovement(MotorMovement(180, 16), ServoMotor(0, 0, 8, 6.5))
PULL_WAREHOUSE = ArmMovement(servo=ServoMotor(0, 0, 8, 16))


def recognition_orchard_tree(arm: ArmService):
    """
    调整为识别姿态（看树）
    """
    arm.control(ArmMovement(MotorMovement(180, 16), ServoMotor(0, 0, 0, 15)))
    arm.control(ArmMovement(MotorMovement(180, 24), ServoMotor(-175, 0, 0, 20)))
    arm.control(ArmMovement(servo=ServoMotor(-175, 0, 0, 20)))
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


def put_fruit_into_basket(arm: ArmService, box_number: int) -> None:
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

    arm.control(ArmMovement(MotorMovement(arm_pos, 16), ServoMotor(0, 0, telescopic, 6.5)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 16), ServoMotor(0, 0, telescopic, 10)))
    time.sleep(0.5)


def grab_basket_to_warehouse(arm: ArmService, box_number: int) -> None:
    """将指定框子放到面前的仓库中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 30
        servo_params = (-56, -90, 15)
    elif box_number == 2:
        arm_pos = 0
        servo_params = (90, -92, 12)
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

def grab_fruit_on_tree(arm_service, move_service, location_on_tree: FruitLocationOnTree):
    CONFIGS = {
        FruitLocationOnTree.TOP_LEFT:    (185, 27, 15, 0.34, 0, 10, 12),
        FruitLocationOnTree.TOP_CENTER:  (180, 25, 0, 0.2, 0, 0, 2),
        FruitLocationOnTree.TOP_RIGHT:   (175, 27, -15, 0.34, 0, 10, 12),
        FruitLocationOnTree.BOTTOM_LEFT: (200, 32, 15, 0.34, -10, 0, 11),
        FruitLocationOnTree.BOTTOM_CENTER:(180, 32, 0, 0.2, 0, 0, 9),
        FruitLocationOnTree.BOTTOM_RIGHT: (160, 32, -15, 0.34, 10, 0, 11)
    }

    match location_on_tree:
        case loc if loc in CONFIGS:
            arm_pos, arm_lift, rotate, line, rotary, nod, telescopic = CONFIGS[loc]
        case _:
            raise RuntimeError("不支持的水果位置")

    # 左上水果
    # 准备抓
    arm_service.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(0, 0, 0, 12)))
    move_service.rotate(rotate)
    move_service.line(line)
    arm_service.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(-170 - rotary, 0, telescopic, 12)))
    # 抓取
    arm_service.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(-170 - rotary, nod, telescopic, 12)))
    time.sleep(0.5)
    arm_service.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(-170 - rotary, nod, telescopic, 6.5)))
    time.sleep(0.5)
    arm_service.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(-170 - rotary, 0, telescopic, 6.5)))
    time.sleep(1)
    # 抓离
    arm_service.control(ArmMovement(MotorMovement(arm_pos, arm_lift), ServoMotor(-170 - rotary, 0, 0, 6.5)))
    move_service.line(-line)
    move_service.rotate(-rotate)
    arm_service.control(ArmMovement(MotorMovement(180, 18), ServoMotor(0, 0, 0, 6.5)))
