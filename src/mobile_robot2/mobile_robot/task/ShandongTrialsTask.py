from py_trees.composites import Sequence


class ShandongTrialsTask(Sequence):
    def __init__(self):
        super().__init__("Shandong Trials Task", True)

        self.add_children([])