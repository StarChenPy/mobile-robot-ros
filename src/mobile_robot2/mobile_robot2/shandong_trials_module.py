import time

import py_trees
import rclpy
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from rclpy.node import Node

from .task.ShandongTrialsTask import ShandongTrialsTask


class ShandongTrialsModule(Node):
    def __init__(self):
        super().__init__("shandong_trials_module")

        py_trees.logging.level = py_trees.logging.Level.DEBUG
        tree = py_trees.trees.BehaviourTree(ShandongTrialsTask())
        tree.setup(15, node=self)

        while True:
            tree.tick()
            print(tree.root.status)
            print(Blackboard.get("/robot_data/raw"))
            time.sleep(1)


def main():
    rclpy.init()

    node = ShandongTrialsModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
