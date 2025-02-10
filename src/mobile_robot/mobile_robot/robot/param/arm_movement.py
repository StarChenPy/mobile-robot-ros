from ..data_type import *


class ArmMovementParam(Enum):
    # 基础动作
    RESET = ArmMovement(MotorMovement(-1, -1), ServoMotor(0, 0, 0, 3.5))
    MOVING = ArmMovement(MotorMovement(0, 15), ServoMotor(0, 9, 0, 3.5))
    BASKET_MOVING = ArmMovement(MotorMovement(180, 15), ServoMotor(0, 0, 0, 3.5))

    # 抓篮子用
    READY_GRAB_BASKET_1 = ArmMovement(MotorMovement(90, 0.5), ServoMotor(0, 7, 0, 24))
    READY_GRAB_BASKET_2 = ArmMovement(MotorMovement(90, 0.5), ServoMotor(0, 7, 10, 24))
    GRAB_BASKET = ArmMovement(servo=ServoMotor(0, 7, 10, 19))

    # 放篮子动作
    READY_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 19))
    PUT_BASKET_CENTER = ArmMovement(MotorMovement(0, 10), ServoMotor(90, -90, 10, 24))
    FINISH_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 24))

    # 识别动作
    READY_RECOGNITION_GRAPE = ArmMovement(MotorMovement(-90, 0.5), ServoMotor(0, -20, 0, 10))
    RECOGNITION_GRAPE = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, -20, 0, 10))

    # 抓葡萄（上）
    READY_GRAB_GRAPE = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, -20, 5, 10))
    GRAB_GRAPE = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, -20, 6, 0))

    # 抓苹果（上）
    READY_GRAB_APPLE = ArmMovement(MotorMovement(-90, 24), ServoMotor(0, 0, 0, 20))
    GRAB_APPLE = ArmMovement(MotorMovement(-90, 24), ServoMotor(0, 0, 9, 7))
    GRAB_APPLE_END = ArmMovement(MotorMovement(0, 10), ServoMotor(0, 0, 9, 7))

    # 放水果到框子动作
    READY_PUT_FRUIT_INTO_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(0, -90, 10, 0))
    PUT_FRUIT_INTO_BASKET = ArmMovement(servo=ServoMotor(0, -90, 10, 10))