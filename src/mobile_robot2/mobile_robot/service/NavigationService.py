from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.blackboard import Blackboard
from py_trees.decorators import Decorator

from web_message_transform_ros2.msg import RobotData
from ..function.CorrectiveFunction import CorrectiveOdomFunction
from ..function.InitOdomFunction import InitYawFunction
from ..model.CorrectivePoint import CorrectivePoint
from ..model.NavigationPoint import NavigationPoint
from ..ros_client.NavigationActionClient import NavigationActionClient
from ..ros_client.TopicSubscriber import robot_data_sub
from ..util import Math


ROTATION_ACCELERATION = 3
ROTATION_DECELERATION = 3


class GenerateNavigationPathsService(Decorator):
    def __init__(self, points: list[NavigationPoint], speed):
        super().__init__("Generate Navigation Paths Service", robot_data_sub)
        self.points = points
        self.speed = speed
        self.step = Sequence("Navigation Sequence", True)

    def __navigation_handle(self, path: list, reverse=False):
        self.step.add_child(NavigationActionClient(path, self.speed, self.speed * 5, ROTATION_ACCELERATION, ROTATION_DECELERATION, reverse))

    def initialise(self) -> None:
        self.step.children.clear()

    def update(self) -> Status:
        if self.decorated.status == Status.SUCCESS:

            path = []
            previous_point = None

            for point in self.points:
                if isinstance(point, CorrectivePoint):
                    if path:
                        path.append(point)
                        self.__navigation_handle(path)
                    elif Blackboard.get("odom/init"):
                        self.__navigation_handle([point])
                    else:
                        self.step.add_child(InitYawFunction(point.yaw))

                    self.step.add_child(CorrectiveOdomFunction(point))
                    path = []
                    continue

                if previous_point is None:
                    robot_data: RobotData = Blackboard.get("robot_data/raw").odom
                    odom = robot_data.odom
                    previous_point = NavigationPoint(odom.x, odom.y, odom.w)

                if previous_point.yaw is not None and Math.is_behind(previous_point, point, 45):
                    # 如果这个点位在上个点位的后面，就倒车回去
                    if path:
                        self.__navigation_handle(path)
                        path = []
                    if point.yaw is None:
                        point.yaw = previous_point.yaw
                    self.__navigation_handle([point])
                else:
                    path.append(point)

                previous_point = point

            if path:
                self.__navigation_handle(path)

            Blackboard.set("navigation/sequence", self.step)

        return self.decorated.status


class RunNavigationPathsService(Behaviour):
    def __init__(self):
        super().__init__("Run Navigation Paths Service")

    def update(self) -> Status:
        seq: Sequence = Blackboard.get("navigation/sequence")

        if seq is not None:
            seq.tick_once()
            return seq.status

        return Status.FAILURE


class NavigationService(Sequence):
    def __init__(self, points: list[NavigationPoint], speed):
        super().__init__("Navigation Service", True)
        self.add_children([
            GenerateNavigationPathsService(points, speed),
            RunNavigationPathsService()
        ])
