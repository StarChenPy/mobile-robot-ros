import time

from ..popo.ArmMovement import ArmMovement
from ..popo.Direction import Direction
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService

# 基础动作
MOVING = ArmMovement(MotorMovement(0, 18), ServoMotor(0, 0, 3, 7))
TEST = ArmMovement(servo=ServoMotor(0, 0, 3, 20))

# 识别果仓中的水果动作
RECOGNITION_WAREHOUSE = ArmMovement(MotorMovement(175, 15), ServoMotor(0, -90, 14, 20))

# 放水果到果仓
READY_PULL_WAREHOUSE = ArmMovement(MotorMovement(180, 10), ServoMotor(0, 0, 8, 7))
PULL_WAREHOUSE = ArmMovement(MotorMovement(180, 10), ServoMotor(0, 0, 8, 16))


def recognition_orchard(arm: ArmService, direction: Direction.LEFT or Direction.RIGHT):
    """调整为识别姿态"""

    if direction == Direction.LEFT:
        arm_pos = 90
    elif direction == Direction.RIGHT:
        arm_pos = -90
    else:
        raise ValueError("不可用的Direction")

    arm.control(ArmMovement(MotorMovement(arm_pos, 12), ServoMotor(0, 0, 3, 7)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 12), ServoMotor(0, -90, 15, 20)))


def put_fruit_into_basket(arm: ArmService, box_number: int) -> None:
    """将水果放置到指定框子中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 28
        telescopic = 2.5
    elif box_number == 2:
        arm_pos = 0
        telescopic = 1
    elif box_number == 3:
        arm_pos = -30
        telescopic = 2.5
    else:
        raise ValueError("篮子编号无效")

    arm.control(ArmMovement(MotorMovement(arm_pos, 18), ServoMotor(0, 0, telescopic, 7)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 18), ServoMotor(0, 0, telescopic, 10)))
    time.sleep(1)


def grab_basket_to_warehouse(arm: ArmService, box_number: int) -> None:
    """将指定框子放到面前的仓库中"""

    # 根据框号确定参数
    if box_number == 1:
        arm_pos = 28
        servo_params = (-61, -87, 14.5)
    elif box_number == 2:
        arm_pos = 0
        servo_params = (90, -87, 11.7)
    elif box_number == 3:
        arm_pos = -30
        servo_params = (61, -87, 14.5)
    else:
        raise ValueError("篮子编号无效")

    rotary, nod, telescopic = servo_params

    # 准备抓
    arm.control(ArmMovement(MotorMovement(0, 10), ServoMotor(0, 0, 3, 7)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 10), ServoMotor(rotary, nod, telescopic, 25)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 14.5), ServoMotor(rotary, nod, telescopic, 25)))
    time.sleep(1)
    # 夹合
    arm.control(ArmMovement(MotorMovement(arm_pos, 14.5), ServoMotor(rotary, nod, telescopic, 20)))
    time.sleep(1)
    # 提起
    arm.control(ArmMovement(MotorMovement(arm_pos, 10), ServoMotor(rotary, nod, telescopic, 20)))
    arm.control(ArmMovement(MotorMovement(arm_pos, 10), ServoMotor(0, 0, 5, 20)))

    # 放下（公共部分）
    arm.control(ArmMovement(MotorMovement(180, 5), ServoMotor(0, 0, 5, 20)))
    arm.control(ArmMovement(MotorMovement(180, 20), ServoMotor(0, 0, 5, 20)))
    arm.control(ArmMovement(MotorMovement(180, 20), ServoMotor(0, 0, 5, 25)))
    time.sleep(2)
    # 结束（公共部分）
    arm.control(ArmMovement(MotorMovement(0, 18), ServoMotor(0, 0, 3, 7)))