from mobile_robot.mobile_robot.service.ArmService import ArmService
from mobile_robot.mobile_robot.service.MoveService import MoveService
from mobile_robot.mobile_robot.service.RobotService import RobotService
from mobile_robot.mobile_robot.service.SensorService import SensorService
from mobile_robot.mobile_robot.service.VisionService import VisionService
from mobile_robot.mobile_robot.util.Logger import Logger
from mobile_robot.mobile_robot.util.Singleton import singleton


@singleton
class TaskBController:
    def __init__(self, node):
        self.logger = Logger()

        self.node = node
        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

    def run(self):
        for i in range(1, 11):
            # 动态获取任务，拒绝重复代码
            task_method = getattr(self, f"task_{i}", None)
            if callable(task_method):
                self.logger.info(f"开始执行任务B-任务{i}")
                task_method()
                self.logger.info(f"任务B-任务{i}完成")
            else:
                self.logger.error(f"任务B-任务{i}不存在")

    def task_1(self):
        self.arm.back_origin()

        self.move.my_navigation("yellow_station_2")

