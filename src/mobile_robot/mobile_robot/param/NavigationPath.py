from .NavigationPoint import *

B_MODULE_4 = (WAREHOUSE_ENTER_3_POINT,
              WAREHOUSE_ENTER_2_POINT,
              WAREHOUSE_ENTER_1_POINT,
              WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
              START_ENTER_POINT,
              START_POINT)

B_MODULE_5 = (START_CORRECTIVE_POINT,
              START_ENTER_POINT,
              WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
              WAREHOUSE_ENTER_1_POINT,
              WAREHOUSE_ENTER_2_POINT,
              WAREHOUSE_ENTER_3_POINT,
              WAREHOUSE_1_POINT)

B_MODULE_6 = (ORCHARD_CORRIDOR_ENTER_1_POINT,
              START_ENTER_POINT,
              START_POINT)

START_TO_ORCHARD_ENTER_1 = (START_CORRECTIVE_POINT,
                            START_ENTER_POINT,
                            ORCHARD_CORRIDOR_ENTER_1_POINT,
                            ORCHARD_CORRIDOR_ENTER_1_CORRECTIVE_POINT)

START_TO_ORCHARD_1 = (START_CORRECTIVE_POINT,
                      START_ENTER_POINT,
                      ORCHARD_CORRIDOR_ENTER_1_POINT,
                      ORCHARD_CORRIDOR_ENTER_1_CORRECTIVE_POINT,
                      ORCHARD_1_POINT)

WAREHOUSE_TO_ORCHARD_2 = (WAREHOUSE_ENTER_3_POINT,
                          WAREHOUSE_ENTER_2_POINT,
                          WAREHOUSE_ENTER_1_POINT,
                          WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                          START_ENTER_POINT,
                          ORCHARD_CORRIDOR_ENTER_1_POINT,
                          ORCHARD_2_POINT)

WAREHOUSE_TO_ORCHARD_3 = (WAREHOUSE_ENTER_3_POINT,
                          WAREHOUSE_ENTER_2_POINT,
                          WAREHOUSE_ENTER_1_POINT,
                          WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                          START_ENTER_POINT,
                          ORCHARD_CORRIDOR_ENTER_1_POINT,
                          ORCHARD_CORRIDOR_EXIT_1_CORRECTIVE_POINT,
                          ORCHARD_3_POINT)

WAREHOUSE_TO_ORCHARD_4 = (WAREHOUSE_ENTER_3_POINT,
                          WAREHOUSE_ENTER_2_POINT,
                          WAREHOUSE_ENTER_1_POINT,
                          WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                          START_ENTER_POINT,
                          ORCHARD_CORRIDOR_ENTER_2_POINT,
                          ORCHARD_4_POINT)

WAREHOUSE_TO_ORCHARD_5 = (WAREHOUSE_ENTER_3_POINT,
                          WAREHOUSE_ENTER_2_POINT,
                          WAREHOUSE_ENTER_1_POINT,
                          WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                          START_ENTER_POINT,
                          ORCHARD_CORRIDOR_ENTER_2_POINT,
                          ORCHARD_5_POINT)

B_MODULE_11 = (START_CORRECTIVE_POINT,
               START_ENTER_POINT,
               WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
               WAREHOUSE_ENTER_1_POINT,
               WAREHOUSE_ENTER_2_POINT,
               WAREHOUSE_ENTER_3_POINT,
               WAREHOUSE_1_POINT)

B_MODULE_12 = (ORCHARD_CORRIDOR_ENTER_1_POINT,
               START_ENTER_POINT,
               WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
               WAREHOUSE_ENTER_1_POINT,
               WAREHOUSE_ENTER_2_POINT,
               WAREHOUSE_ENTER_3_POINT,
               WAREHOUSE_1_POINT)

ORCHARD_CORRIDOR_ENTER_1 = (ORCHARD_CORRIDOR_EXIT_1_POINT,)

ORCHARD_CORRIDOR_ENTER_2 = (ORCHARD_CORRIDOR_EXIT_2_POINT,
                            ORCHARD_CORRIDOR_ENTER_2_POINT)

ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT = (ORCHARD_CORRIDOR_ENTER_1_POINT,
                                           WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                                           WAREHOUSE_ENTER_1_POINT,
                                           WAREHOUSE_ENTER_2_POINT,
                                           WAREHOUSE_ENTER_3_POINT,
                                           WAREHOUSE_1_POINT)

ORCHARD_CORRIDOR_2_TO_WAREHOUSE_1_POINT = (ORCHARD_CORRIDOR_ENTER_2_POINT,
                                           WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                                           WAREHOUSE_ENTER_1_POINT,
                                           WAREHOUSE_ENTER_2_POINT,
                                           WAREHOUSE_ENTER_3_POINT,
                                           WAREHOUSE_1_POINT)

WAREHOUSE_TO_ORCHARD_ENTER_1 = (WAREHOUSE_ENTER_3_POINT,
                                WAREHOUSE_ENTER_2_POINT,
                                WAREHOUSE_ENTER_1_POINT,
                                WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                                START_ENTER_POINT,
                                ORCHARD_CORRIDOR_ENTER_1_POINT,
                                ORCHARD_CORRIDOR_ENTER_1_CORRECTIVE_POINT)

ENTER_2_POINT_TO_WAREHOUSE_1_POINT = (ORCHARD_CORRIDOR_ENTER_2_POINT,
                                      START_ENTER_POINT,
                                      WAREHOUSE_ENTER_0_CORRECTIVE_POINT,
                                      WAREHOUSE_ENTER_1_POINT,
                                      WAREHOUSE_ENTER_2_POINT,
                                      WAREHOUSE_ENTER_3_POINT,
                                      WAREHOUSE_1_POINT)

EXIT_1_TO_EXIT_2 = (ORCHARD_CORRIDOR_EXIT_1_CORRECTIVE_POINT,
                    ORCHARD_CORRIDOR_EXIT_2_POINT)

EXIT_2_TO_ENTER_2 = (ORCHARD_CORRIDOR_ENTER_2_POINT,)
