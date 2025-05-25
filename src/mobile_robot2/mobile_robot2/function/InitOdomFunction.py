import math

from py_trees.decorators import Decorator
from py_trees.common import Status
from py_trees.blackboard import Blackboard
from chassis_msgs.srv import ResetOdom

from ..model.NavigationPoint import NavigationPoint
from ..ros_client.OdomService import OdomService


class InitAllFunction(Decorator):
    def __init__(self, pose: NavigationPoint):
        req = ResetOdom.Request()
        req.clear_mode = 0
        req.x = float(pose.x)
        req.y = float(pose.y)
        radian = math.radians(pose.yaw)
        req.theta = float(radian)

        Blackboard.set("/odom/request", req)

        super().__init__("Init Odom All Function", OdomService())

    def update(self) -> Status:
        status = self.decorated.status

        if status == Status.SUCCESS:
            result = Blackboard.get("/odom/response")

            if result.success:
                self.logger.debug("重置成功")
                Blackboard.set("/odom/init", True)
                return Status.SUCCESS
            else:
                self.logger.error("重置失败")
                return Status.FAILURE

        return status


class InitLocationFunction(Decorator):
    def __init__(self, x: float, y: float):
        super().__init__("Init Odom Location Function", InitAllFunction(NavigationPoint(x, y, 0)))

    def update(self) -> Status:
        return self.decorated.status

class InitYawFunction(Decorator):
    def __init__(self, yaw: float):
        super().__init__("Init Odom Location Function", InitAllFunction(NavigationPoint(0, 0, yaw)))

    def update(self) -> Status:
        return self.decorated.status
