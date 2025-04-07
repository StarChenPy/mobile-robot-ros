from ..util.ConfigAndParam import ConfigAndParam

param = ConfigAndParam()

B_MODULE_4 = (param.get_navigation_point("warehouse_enter_3_point"),
              param.get_navigation_point("warehouse_enter_2_point"),
              param.get_navigation_point("warehouse_enter_1_point"),
              param.get_navigation_point("warehouse_enter_0_corrective_point"),
              param.get_navigation_point("start_enter_point"),
              param.get_navigation_point("start_point"))

B_MODULE_5 = (param.get_navigation_point("start_corrective_point"),
              param.get_navigation_point("start_enter_point"),
              param.get_navigation_point("warehouse_enter_0_corrective_point"),
              param.get_navigation_point("warehouse_enter_1_point"),
              param.get_navigation_point("warehouse_enter_2_point"),
              param.get_navigation_point("warehouse_enter_3_point"),
              param.get_navigation_point("warehouse_1_point_range"))

B_MODULE_6 = (param.get_navigation_point("orchard_corridor_enter_1_point"),
              param.get_navigation_point("warehouse_enter_0_corrective_point"),
              param.get_navigation_point("start_enter_point"),
              param.get_navigation_point("start_point"))

START_TO_ORCHARD_ENTER_1 = (param.get_navigation_point("start_corrective_point"),
                            param.get_navigation_point("start_enter_point"),
                            # 开口向下专属
                            param.get_navigation_point("warehouse_enter_0_corrective_point"),
                            param.get_navigation_point("orchard_corridor_enter_1_point"),
                            param.get_navigation_point("orchard_corridor_start_1_corrective_point"))

START_TO_ORCHARD_1 = (param.get_navigation_point("start_corrective_point"),
                      param.get_navigation_point("start_enter_point"),
                      # 开口向下专属
                      param.get_navigation_point("warehouse_enter_0_corrective_point"),
                      param.get_navigation_point("orchard_corridor_enter_1_point"),
                      param.get_navigation_point("orchard_corridor_start_1_corrective_point"),
                      param.get_navigation_point("orchard_1_point"))

ORCHARD_1_TO_ORCHARD_2 = (param.get_navigation_point("orchard_2_point"),)

ORCHARD_2_TO_ORCHARD_3 = (param.get_navigation_point("orchard_3_point"), )

ORCHARD_3_TO_ORCHARD_4 = (param.get_navigation_point("orchard_corridor_enter_1_point"),
                          param.get_navigation_point("orchard_corridor_enter_2_point"),
                          param.get_navigation_point("orchard_corridor_start_2_corrective_point"),
                          param.get_navigation_point("orchard_4_point"))

ORCHARD_4_TO_ORCHARD_5 = (param.get_navigation_point("orchard_5_point"),)

B_MODULE_11 = (param.get_navigation_point("start_corrective_point"),
               param.get_navigation_point("start_enter_point"),
               param.get_navigation_point("warehouse_enter_0_corrective_point"),
               param.get_navigation_point("warehouse_enter_1_point"),
               param.get_navigation_point("warehouse_enter_2_point"),
               param.get_navigation_point("warehouse_enter_3_point"),
               param.get_navigation_point("warehouse_1_point"))

B_MODULE_12 = (param.get_navigation_point("orchard_corridor_enter_1_point"),
               param.get_navigation_point("warehouse_enter_0_corrective_point"),
               param.get_navigation_point("warehouse_enter_1_point"),
               param.get_navigation_point("warehouse_enter_2_point"),
               param.get_navigation_point("warehouse_enter_3_point"),
               param.get_navigation_point("warehouse_1_point"))

B_MODULE_13 = (param.get_navigation_point("start_corrective_point"),
               param.get_navigation_point("start_enter_point"),
               param.get_navigation_point("warehouse_enter_0_corrective_point"),
               param.get_navigation_point("warehouse_enter_1_point"),
               param.get_navigation_point("warehouse_enter_2_point"),
               param.get_navigation_point("warehouse_enter_3_point"),
               param.get_navigation_point("warehouse_1_point"))

ORCHARD_CORRIDOR_ENTER_1 = (param.get_navigation_point("orchard_corridor_exit_1_point"),)

ORCHARD_CORRIDOR_ENTER_2 = (param.get_navigation_point("orchard_corridor_exit_2_point"),
                            param.get_navigation_point("orchard_corridor_enter_2_point"))

ORCHARD_CORRIDOR_2_TO_WAREHOUSE = (param.get_navigation_point("orchard_corridor_enter_2_point"),
                                   param.get_navigation_point("warehouse_enter_0_corrective_point"),
                                   param.get_navigation_point("warehouse_enter_1_point"),
                                   param.get_navigation_point("warehouse_enter_2_point"),
                                   param.get_navigation_point("warehouse_enter_3_point"),
                                   param.get_navigation_point("warehouse_corrective_point"))

WAREHOUSE_TO_ORCHARD_ENTER_1 = (param.get_navigation_point("warehouse_enter_3_point"),
                                param.get_navigation_point("warehouse_enter_2_point"),
                                param.get_navigation_point("warehouse_enter_1_point"),
                                param.get_navigation_point("warehouse_enter_0_corrective_point"),
                                param.get_navigation_point("start_enter_point"),
                                param.get_navigation_point("orchard_corridor_enter_1_point"),
                                param.get_navigation_point("orchard_corridor_start_1_corrective_point"))

ENTER_2_POINT_TO_WAREHOUSE_1_POINT = (param.get_navigation_point("orchard_corridor_enter_2_point"),
                                      param.get_navigation_point("start_enter_point"),
                                      param.get_navigation_point("warehouse_enter_0_corrective_point"),
                                      param.get_navigation_point("warehouse_enter_1_point"),
                                      param.get_navigation_point("warehouse_enter_2_point"),
                                      param.get_navigation_point("warehouse_enter_3_point"),
                                      param.get_navigation_point("warehouse_1_point"))

EXIT_1_TO_EXIT_2 = (param.get_navigation_point("orchard_corridor_exit_1_corrective_point"),
                    param.get_navigation_point("orchard_corridor_exit_2_point"))

EXIT_2_TO_ENTER_2 = (param.get_navigation_point("orchard_corridor_enter_2_point"),)
