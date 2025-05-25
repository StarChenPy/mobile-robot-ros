from py_trees.composites import Sequence

from ..service.NavigationService import NavigationService
from ..util.NavigationPointParam import NavigationPointParam

param = NavigationPointParam("shandong_trials_navigation_point.yml")
START_TO_TREE_1 = [param.get_navigation_point("start_corrective_point"),
                   param.get_navigation_point("start_enter_point"),
                   param.get_navigation_point("orchard_enter_a_1"),
                   param.get_navigation_point("orchard_enter_a_2"),
                   param.get_navigation_point("orchard_enter_a_3"),
                   param.get_navigation_point("orchard_enter_a_fruit_tree_1")]


class ShandongTrialsTask(Sequence):
    def __init__(self):
        super().__init__("Shandong Trials Task", True)
        self.add_children([NavigationService(START_TO_TREE_1, 0.4)])