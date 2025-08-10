import enum


class Direction(enum.IntEnum):
    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3

    def invert(self):
        # 0<->1 和 2<->3 通过异或1实现反转
        return Direction(self.value ^ 1)
