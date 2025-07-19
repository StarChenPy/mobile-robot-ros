from mobile_robot.mobile_robot.util.Singleton import singleton


@singleton
class TaskDController:
    def __init__(self, node):
        self.node = node
