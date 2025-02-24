import enum


class MotorCmd(enum.Enum):
    READ_FEEDBACK = 0
    BACK_ORIGIN = 1
    SET_POSITION = 2
    DISABLE = 3