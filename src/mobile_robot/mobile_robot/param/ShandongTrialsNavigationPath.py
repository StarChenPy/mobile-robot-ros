from ..util.NavigationPointParam import NavigationPointParam

param = NavigationPointParam("shandong_trials_navigation_point.yml")

START_TO_TREE_1 = (param.get_navigation_point("start_corrective_point"),
                   param.get_navigation_point("start_enter_point"),
                   param.get_navigation_point("orchard_enter_a_1"),
                   param.get_navigation_point("orchard_enter_a_2"),
                   param.get_navigation_point("orchard_enter_a_3"),
                   param.get_navigation_point("orchard_enter_a_fruit_tree_1"))

TREE_1_TO_TREE_2 = (param.get_navigation_point("orchard_enter_a_fruit_tree_2"), )

TREE_2_TO_TREE_2 = (param.get_navigation_point("orchard_enter_a_3"),
                    param.get_navigation_point("orchard_enter_a_2"),
                    param.get_navigation_point("orchard_enter_a_1"),
                    param.get_navigation_point("orchard_enter_b_1"),
                    param.get_navigation_point("orchard_enter_b_2"),
                    param.get_navigation_point("orchard_enter_b_3"),
                    param.get_navigation_point("orchard_enter_b_4"),
                    param.get_navigation_point("orchard_enter_b_fruit_tree_2"))

TREE_2_TO_TREE_3 = (param.get_navigation_point("orchard_enter_b_3"),
                    param.get_navigation_point("orchard_enter_b_fruit_tree_3"),)

TREE_3_TO_TREE_4 = (param.get_navigation_point("orchard_enter_b_fruit_tree_4"), )

TREE_4_TO_TREE_START = (param.get_navigation_point("orchard_enter_b_2"),
                        param.get_navigation_point("orchard_enter_b_1"),
                        param.get_navigation_point("start_enter_point"),
                        param.get_navigation_point("start_point"),)