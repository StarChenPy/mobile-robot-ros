from py_trees_ros.service_clients import FromConstant, FromBlackboard

from camera_orbbec2.srv import ReqImage
from position_motor_ros2.srv import CtrlImpl
from base_motion_ros2.srv import BaseMotion
from chassis_msgs.srv import ResetOdom
from user_sensor_msgs.srv import SensorService


"""拍摄服务 响应时间约100ms-200ms"""
take_image_service = FromConstant("Take image Service",
                                  ReqImage,
                                  "/camera/req_image",
                                  ReqImage.Request(cmd=0, sync_snap=True),
                                  "camera/raw",
                                  wait_for_server_timeout_sec=10)


"""升降电机服务"""
lift_motor_service = FromBlackboard("Lift Motor Service",
                                    CtrlImpl,
                                    "/position_motor/lift_motor/ctrl",
                                    "lift_motor/request",
                                    "lift_motor/response",
                                    wait_for_server_timeout_sec=10)


"""旋转电机服务"""
rotate_motor_service = FromBlackboard("Rotate Motor Service",
                                      CtrlImpl,
                                      "/position_motor/rotate_motor/ctrl",
                                      "rotate_motor/request",
                                      "rotate_motor/response",
                                      wait_for_server_timeout_sec=10)


"""运动服务"""
motion_service = FromBlackboard("Motion Service",
                                BaseMotion,
                                "/base_motion",
                                "base_motion/request",
                                "base_motion/response",
                                wait_for_server_timeout_sec=10)


"""里程计服务"""
odom_service = FromBlackboard("Odom Service",
                              ResetOdom,
                              "/chassis/reset_odom",
                              "odom/request",
                              "odom/response",
                              wait_for_server_timeout_sec=10)


"""传感器矫正服务"""
sensor_service = FromBlackboard("Sensor Service",
                              SensorService,
                              "/user/sensor_service_cmd",
                              "sensor/request",
                              "sensor/response",
                              wait_for_server_timeout_sec=10)
