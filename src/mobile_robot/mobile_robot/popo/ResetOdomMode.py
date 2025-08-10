import enum


class ResetOdomMode(enum.Enum):
    RESET_ALL = 0
    RESET_POSE = 1
    RESET_YAW = 2
