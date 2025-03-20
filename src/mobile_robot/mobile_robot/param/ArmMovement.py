from ..popo.ArmMovement import ArmMovement
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor


# 基础动作
RESET = ArmMovement(MotorMovement(-1, -1), ServoMotor(0, 0, 3, 5))
TEST = ArmMovement(servo=ServoMotor(0, 0, 0, 20))
MOVING = ArmMovement(MotorMovement(0, 15), ServoMotor(0, 9, 3, 7))
BASKET_MOVING = ArmMovement(MotorMovement(180, 15), ServoMotor(0, 0, 3, 5))

# 篮子动作
READY_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 19))

PUT_BASKET_1 = ArmMovement(MotorMovement(20, 10), ServoMotor(90, -90, 10, 24))
PUT_BASKET_2 = ArmMovement(MotorMovement(0, 10), ServoMotor(90, -90, 10, 24))
PUT_BASKET_3 = ArmMovement(MotorMovement(-20, 10), ServoMotor(90, -90, 10, 24))

FINISH_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 24))

# 识别动作
RECOGNITION_ORCHARD_RIGHT = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, -30, 3, 20))
RECOGNITION_ORCHARD_LEFT = ArmMovement(MotorMovement(90, 25), ServoMotor(0, -30, 3, 20))
RECOGNITION_WAREHOUSE = ArmMovement(MotorMovement(175, 15), ServoMotor(0, -90, 14, 20))

READY_GRAB_APPLE = ArmMovement(MotorMovement(-90, 16), ServoMotor(0, -20, 0, 23))
# 抓苹果（上）
READY_GRAB_APPLE_TALL = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, 0, 10, 23))
GRAB_APPLE_TALL = ArmMovement(MotorMovement(-90, 25), ServoMotor(0, 0, 10, 7))
# 抓苹果（中）
READY_GRAB_APPLE_MIDDLE = ArmMovement(MotorMovement(-90, 26), ServoMotor(0, -20, 12.5, 23))
GRAB_APPLE_MIDDLE = ArmMovement(MotorMovement(-90, 26), ServoMotor(0, -20, 12.5, 7))
GRAB_APPLE_MIDDLE_END = ArmMovement(MotorMovement(-90, 26), ServoMotor(0, -30, 7, 7))
# 抓苹果（下）
READY_GRAB_APPLE_LOW = ArmMovement(MotorMovement(-90, 27.5), ServoMotor(0, -45, 15, 23))
GRAB_APPLE_LOW = ArmMovement(MotorMovement(-90, 27.5), ServoMotor(0, -45, 15, 7))
GRAB_APPLE_LOW_END = ArmMovement(MotorMovement(-90, 20), ServoMotor(0, -45, 7, 7))
GRAB_APPLE_END = ArmMovement(MotorMovement(0, 10), ServoMotor(0, 0, 7, 7))

# 放水果到果仓
READY_PULL_GUO_CANG = ArmMovement(MotorMovement(180, 10), ServoMotor(0, 0, 8, 7))
PULL_GUO_CANG = ArmMovement(MotorMovement(180, 10), ServoMotor(0, 0, 8, 20))

# 放水果到框子动作
READY_PUT_FRUIT_INTO_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(0, -90, 10, 0))
PUT_FRUIT_INTO_BASKET_1 = ArmMovement(MotorMovement(-20, 0.5), ServoMotor(0, -90, 10, 10))
PUT_FRUIT_INTO_BASKET_2 = ArmMovement(servo=ServoMotor(0, -90, 10, 10))
PUT_FRUIT_INTO_BASKET_3 = ArmMovement(MotorMovement(20, 0.5), ServoMotor(0, -90, 10, 10))
