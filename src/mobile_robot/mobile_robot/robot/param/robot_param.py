from collections import namedtuple
from enum import Enum, auto

# 坐标点定义
Point = namedtuple('Point', ['x', 'y', 'w'])

# 枚举定义：位置名称
class PathPoint(Enum):
    起始区 = Point(-34, -35, -90)
    识别台 = Point(-348, -35, -90)
    标识A = Point(-215, -35, 0)
    标识B = Point(-301, -35, 0)
    采摘入口 = Point(-167, -362, 0)
    采摘A = Point(-167, -318, 0)
    采摘B = Point(-100, -185, 0)
    采摘C = Point(-97, -238, 0)
    采摘D = Point(-35, -294, 0)
    结束A = Point(-165, -100, 0)
    结束B = Point(-97, -100, 0)
    结束C = Point(-97, -369, 0)
    结束D = Point(-30, -100, 0)
    A = Point(-64, -35, 0.0)
    B = Point(-235, -35, 0.0)
    C = Point(-235, -200, 0.0)
    D = Point(-235, -365, 0.0)
    E = Point(-165, -365, 0.0)
    F = Point(-165, -210, 0.0)
    G = Point(-165, -110, 0.0)
    H = Point(-96, -211, 0.0)
    I = Point(-96, -440, 0.0)
    J = Point(-96, -365, 0.0)
    K = Point(-30, -365, 0.0)
    L = Point(-30, -110, 0.0)
    M = Point(-330, -35, 0.0)
    N = Point(-301, -300, 0.0)
    test = Point(0, 0, 0)
    test1 = Point(0, 0, 0)
    test2 = Point(0, 50, 0)

# 枚举定义：路径规划
class Path(Enum):
    起始区_识别台 = [PathPoint.A, PathPoint.B, PathPoint.M, PathPoint.识别台]
    识别台_标识A = [PathPoint.M, PathPoint.标识A]
    标识A_标识B = [PathPoint.标识B]
    标识A_采摘入口 = [PathPoint.C, PathPoint.D, PathPoint.采摘入口]
    标识B_采摘入口 = [PathPoint.N, PathPoint.D, PathPoint.采摘入口]
    采摘入口_A_起始区 = [PathPoint.D, PathPoint.C, PathPoint.B, PathPoint.A, PathPoint.起始区]
    采摘入口_B_起始区 = [PathPoint.D, PathPoint.标识B, PathPoint.B, PathPoint.A, PathPoint.起始区]
    采摘入口_采摘A = [PathPoint.采摘A]
    结束A_采摘B = [PathPoint.F, PathPoint.H, PathPoint.采摘B]
    结束B_采摘C = [PathPoint.H, PathPoint.采摘C]
    结束C_采摘D = [PathPoint.K, PathPoint.采摘D]
    结束D_采摘入口 = [
        PathPoint.L, PathPoint.K, PathPoint.J, PathPoint.H,
        PathPoint.F, PathPoint.E, PathPoint.采摘入口
    ]
    test_test2 = [PathPoint.test2]

# 枚举定义：控制参数
class GripperParams(Enum):
    复位 = 3.5
    运动时 = 10.0
    抓取篮子 = 10.0
    放置篮子 = 24.5
    完全打开 = 24.5
    篮子抓车_中 = 15.0
    识别葡萄 = 18.0
    抓取通道葡萄 = 0.0

class TelescopicParams(Enum):
    复位 = 0.0
    运动时 = 0.0
    篮子抓车_中 = 1.0
    平台篮子_抓 = 10.0
    识别台 = 5.0
    识别通道葡萄 = 0.0
    抓取通道葡萄 = 6.5

class GripperRyParams(Enum):
    复位 = 0.0
    运动时 = 90.0
    篮子抓车_中 = -93.0
    平台篮子_抓 = 0.0
    识别台 = -90.0
    识别通道葡萄 = -83.0
    抓取通道葡萄 = -75.0

class GripperRzParams(Enum):
    复位 = 0.0
    运动时 = 0.0
    篮子抓车_中 = 90
    平台篮子_抓 = 0.0

class RotateMotorParams(Enum):
    复位 = 0.0
    运动时 = 0.0
    篮子抓车_中 = 0.0
    平台篮子_抓 = 90
    识别台 = 180
    看葡萄_左 = 90
    看葡萄_右 = -90

class ElevatorMotorParams(Enum):
    复位 = 0.0
    运动时 = 16.0
    篮子抓车_中 = 10.0
    复位_1 = 1.0
    识别台 = 12.0
    识别通道葡萄 = 15.0
    抓取通道葡萄 = 24.0

# 预设识别元素
class PresetElement(Enum):
    Purple_grape = auto()
    Green_grape = auto()
    Yellow_grape = auto()
