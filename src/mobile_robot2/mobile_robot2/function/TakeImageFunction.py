import cv_bridge
from py_trees.decorators import Decorator
from py_trees.common import Status
from py_trees.blackboard import Blackboard
from camera_orbbec2.srv import ReqImage

from ..ros_client.TakeImageService import TakeImageService

class TakeImageFunction(Decorator):
    def __init__(self):
        super().__init__("Take Image Function", TakeImageService())

        self.cv_bridge = cv_bridge.CvBridge()

    def update(self) -> Status:
        result = self.decorated.status

        if result == Status.SUCCESS:
            data: ReqImage.Response = Blackboard.get("camera/raw")

            color = self.cv_bridge.imgmsg_to_cv2(data.image_color, 'bgr8')
            depth = self.cv_bridge.imgmsg_to_cv2(data.image_depth, 'mono16')

            Blackboard.set("camera/depth", depth)
            Blackboard.set("camera/color", color)

        return result

