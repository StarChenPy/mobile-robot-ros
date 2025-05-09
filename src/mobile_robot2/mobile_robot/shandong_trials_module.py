import py_trees
import rclpy
from py_trees.common import Status
from rclpy.node import Node

from .task.ShandongTrialsTask import ShandongTrialsTask


class ShandongTrialsModule(Node):
    def __init__(self):
        super().__init__("shandong_trials_module")

        tree = py_trees.trees.BehaviourTree(ShandongTrialsTask())
        tree.setup(15, node=self)

        while tree.root.status == Status.RUNNING:
            tree.tick()

        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()

    node = ShandongTrialsModule()

    rclpy.spin(node)
