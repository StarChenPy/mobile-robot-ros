import enum

from ..util.data_type import ArmMovement, MotorMovement, ServoMotor


class ArmMovementParam(enum.Enum):
    # 基础动作
    RESET = ArmMovement(MotorMovement(-1, -1), ServoMotor(0, 0, 0, 5))
    TEST = ArmMovement(servo=ServoMotor(0, 0, 0, 20))
    MOVING = ArmMovement(MotorMovement(0, 15), ServoMotor(0, 9, 0, 5))
    BASKET_MOVING = ArmMovement(MotorMovement(180, 15), ServoMotor(0, 0, 0, 5))

    # 抓篮子用
    READY_GRAB_BASKET_1 = ArmMovement(MotorMovement(90, 0.5), ServoMotor(0, 7, 0, 24))
    READY_GRAB_BASKET_2 = ArmMovement(MotorMovement(90, 0.5), ServoMotor(0, 7, 10, 24))
    GRAB_BASKET = ArmMovement(servo=ServoMotor(0, 7, 10, 19))

    # 放篮子动作
    READY_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 19))
    PUT_BASKET_CENTER = ArmMovement(MotorMovement(0, 10), ServoMotor(90, -90, 10, 24))
    FINISH_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 24))

    # 识别动作
    READY_RECOGNITION_ORCHARD = ArmMovement(MotorMovement(-90, 0.5), ServoMotor(0, -30, 0, 20))
    RECOGNITION_ORCHARD = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, -30, 0, 20))
    RECOGNITION_WAREHOUSE = ArmMovement(MotorMovement(175, 15), ServoMotor(0, -90, 14, 20))

    # 抓苹果（上）
    READY_GRAB_APPLE_TALL = ArmMovement(MotorMovement(-90, 20), ServoMotor(0, 0, 0, 23))
    GRAB_APPLE_TALL = ArmMovement(MotorMovement(-90, 26), ServoMotor(0, 0, 8, 7))
    # 抓苹果（中）
    READY_GRAB_APPLE_MIDDLE = ArmMovement(MotorMovement(-90, 15), ServoMotor(0, -50, 13, 23))
    GRAB_APPLE_MIDDLE = ArmMovement(MotorMovement(-90, 24), ServoMotor(0, -50, 13, 7))
    # 抓苹果（下）
    READY_GRAB_APPLE_LOW = ArmMovement(MotorMovement(-90, 15), ServoMotor(0, -50, 13, 23))
    GRAB_APPLE_LOW = ArmMovement(MotorMovement(-90, 28), ServoMotor(0, -60, 14, 7))
    GRAB_APPLE_END = ArmMovement(MotorMovement(0, 10), ServoMotor(0, 0, 7, 7))

    # 放水果到果仓
    READY_PULL_GUO_CANG = ArmMovement(MotorMovement(175, 10), ServoMotor(0, 0, 14, 7))
    PULL_GUO_CANG = ArmMovement(MotorMovement(175, 10), ServoMotor(0, 0, 14, 20))

    # 放水果到框子动作
    READY_PUT_FRUIT_INTO_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(0, -90, 10, 0))
    PUT_FRUIT_INTO_BASKET = ArmMovement(servo=ServoMotor(0, -90, 10, 10))
