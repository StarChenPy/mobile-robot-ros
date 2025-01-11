from ..param import arm_param
import time

from .io_impl import IoImpl
from rclpy.node import Node
from position_motor_ros2.srv import CtrlImpl


class ArmImpl:
    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        self.__io = IoImpl(node)

        self.__srv_rotate_motor = node.create_client(CtrlImpl,'/position_motor/rotate_motor/ctrl')
        self.__srv_lift_motor = node.create_client(CtrlImpl, '/position_motor/lift_motor/ctrl')

        self.__logger.info("[机械臂] 初始化完成")

    #卡爪舵机 旋臂(角度)
    def gripper_rz(self, deg=0, release=False):
        self.__logger.info('[arm 旋臂] 设置角度:{} 释放舵机:{}'.format(deg, release))

        if release:
            self.__io.write_pwm(arm_param.gripper_rz['pin'], 0.0)
            return

        #限位处理
        if deg < arm_param.gripper_rz['min']:
            self.__logger.info('[arm 旋臂] 警告 目标角度:{} 超出最小限位:{}'.format(deg, arm_param.gripper_rz['min']))
            deg = arm_param.gripper_rz['min']
        if deg > arm_param.gripper_rz['max']:
            self.__logger.info('[arm 旋臂] 警告 目标角度:{} 超出最大限位:{}'.format(deg, arm_param.gripper_rz['max']))
            deg = arm_param.gripper_rz['max']
        #角度转duty
        coeff = (arm_param.gripper_rz['deg90_duty'] - arm_param.gripper_rz['zero_duty']) / 90.0
        duty = arm_param.gripper_rz['zero_duty'] + deg * coeff
        self.__io.write_pwm(arm_param.gripper_rz['pin'], duty)

    #卡爪舵机 摆臂(角度)
    def gripper_ry(self, deg=0, release=False):
        self.__logger.info('[arm 摆臂] 设置角度:{} 释放舵机:{}'.format(deg, release))

        if release:
            self.__io.write_pwm(arm_param.gripper_ry['pin'], 0.0)
            return

        #限位处理
        if deg < arm_param.gripper_ry['min']:
            self.__logger.info('[arm 摆臂] 警告 目标角度:{} 超出最小限位:{}'.format(deg, arm_param.gripper_ry['min']))
            deg = arm_param.gripper_ry['min']
        if deg > arm_param.gripper_ry['max']:
            self.__logger.info('[arm 摆臂] 警告 目标角度:{} 超出最大限位:{}'.format(deg, arm_param.gripper_ry['max']))
            deg = arm_param.gripper_ry['max']
        #角度转duty
        coeff = (arm_param.gripper_ry['deg90_duty'] - arm_param.gripper_ry['zero_duty']) / 90.0  # 9.0 - 6.0 / 90.0 = 0.033
        duty = arm_param.gripper_ry['zero_duty'] + deg * coeff  # 6.0 + 60 * 0.033
        #写入

        self.__io.write_pwm(arm_param.gripper_ry['pin'], duty)

    #伸缩(距离cm)
    def telescopic(self, distance=0, release=False):
        self.__logger.info('[arm 伸缩] 设置距离:{}cm 释放舵机:{}'.format(distance, release))

        if release:
            self.__io.write_pwm(arm_param.telescopic_param['pin'], 0)
            return

        #限位处理
        if distance < 0:
            self.__logger.info('[arm 伸缩] 警告 目标距离:{} 超出最小限位:{}'.format(distance, 0))
            distance = 0
        if distance > arm_param.telescopic_param['itinerary']:
            self.__logger.info('[arm 伸缩] 警告 目标距离:{} 超出最大限位:{}'.format(distance, arm_param.telescopic_param['itinerary']))
            distance = arm_param.telescopic_param['itinerary']
        #距离转duty  8.5 - 3.5 / 10   5
        coeff = (arm_param.telescopic_param['max_duty'] - arm_param.telescopic_param['min_duty']) / arm_param.telescopic_param['itinerary'] # 0.5

        duty = arm_param.telescopic_param['max_duty'] - distance * coeff

        self.__io.write_pwm(arm_param.telescopic_param['pin'], duty)

    #卡爪(距离cm)
    def gripper(self, distance=0, release=False):
        self.__logger.info('[arm 卡爪] 设置距离:{}cm 释放舵机:{}'.format(distance, release))

        if release:
            self.__io.write_pwm(arm_param.gripper_param['pin'], 0)
            return

        #限位处理
        if distance < arm_param.gripper_param['min_dis']:
            self.__logger.info('[arm 卡爪] 警告 目标距离:{} 超出最小限位:{}'.format(distance, arm_param.gripper_param['min_dis']))
            distance = arm_param.gripper_param['min_dis']
        if distance > arm_param.gripper_param['max_dis']:
            self.__logger.info('[arm 卡爪] 警告 目标距离:{} 超出最大限位:{}'.format(distance, arm_param.gripper_param['max_dis']))
            distance = arm_param.gripper_param['max_dis']

        #距离转duty
        coeff = (arm_param.gripper_param['max_duty'] - arm_param.gripper_param['min_duty']) / arm_param.gripper_param['itinerary'] # 10.0 - 3.5 / 10.5
        duty = arm_param.gripper_param['min_duty'] + distance / 2.0 * coeff   # 3.5 + 0.62 * dis /2
        self.__logger.info(duty)

        self.__io.write_pwm(arm_param.gripper_param['pin'], duty)

    #设置旋转电机命令[read_feedback, back_origin, set_position, disable]
    def rotate_motor(self, cmd='back_origin', target_deg=0, vel=100, log=True):
        if log:
            self.__logger.info('[arm] 旋转电机 命令:{} 目标角度:{}'.format(cmd, target_deg))
        #参数配置
        max_deg = arm_param.rotate_motor['max_deg']      #最大角度
        min_deg = arm_param.rotate_motor['min_deg']      #最小角度
        origin_param = self.__create_origin_param(arm_param.rotate_motor['origin']['back_vel'], arm_param.rotate_motor['origin']['find_cnt'], arm_param.rotate_motor['origin']['approix_vel'],
                                                  arm_param.rotate_motor['origin']['approix_step'])
        ctrl_param = self.__create_axis_param(arm_param.rotate_motor['ctrl_param']['kp'], arm_param.rotate_motor['ctrl_param']['ti'], arm_param.rotate_motor['ctrl_param']['td'], vel, arm_param.rotate_motor['ctrl_param']['max_acc'],
                                              arm_param.rotate_motor['ctrl_param']['low_pass'], arm_param.rotate_motor['ctrl_param']['ek'], arm_param.rotate_motor['ctrl_param']['steady_clk'])

        #角度转脉冲
        if target_deg > max_deg:
            target_deg = max_deg
        if target_deg < min_deg:
            target_deg = min_deg
        enc_ppi = 1750.0 * 4.0
        target_pose = target_deg * (enc_ppi / 360.0)
        #调用服务
        res = self.__call_position_srv(self.__srv_rotate_motor, cmd, target_pose, origin_param, ctrl_param)
        #反馈中的脉冲转角度
        res.feedback.curr_point = res.feedback.curr_point / (enc_ppi / 360.0)
        return res
    # self.__logger.info(res)

    #等待旋转电机完成
    def rotate_motor_wait_finished(self):
        self.__logger.info('[arm] 旋转电机 等待运动完成.')
        while True:
            time.sleep(0.1)
            res = self.rotate_motor('read_feedback', 0, 0, False)

            if res.feedback.reached:
                self.__logger.info("[arm] 旋转运动完成.")
                break

    def lift_motor(self, cmd, target_cm=0, vel=100.0, log=True):
        if log:
            self.__logger.info('[arm] 升降电机 命令:{} 目标距离:{}cm'.format(cmd, target_cm))
        #参数配置
        origin_param = self.__create_origin_param(arm_param.lift_motor['origin']['back_vel'], arm_param.lift_motor['origin']['find_cnt'], arm_param.lift_motor['origin']['approix_vel'],
                                                  arm_param.lift_motor['origin']['approix_step'])
        ctrl_param = self.__create_axis_param(arm_param.lift_motor['ctrl_param']['kp'], arm_param.lift_motor['ctrl_param']['ti'], arm_param.lift_motor['ctrl_param']['td'], vel, arm_param.lift_motor['ctrl_param']['max_acc'],
                                              arm_param.lift_motor['ctrl_param']['low_pass'], arm_param.lift_motor['ctrl_param']['ek'], arm_param.lift_motor['ctrl_param']['steady_clk'])

        #软限位
        if target_cm > arm_param.lift_motor['max_dis']:
            target_cm = arm_param.lift_motor['max_dis']
        if target_cm < arm_param.lift_motor['min_dis']:
            target_cm = arm_param.lift_motor['min_dis']

        #单位转换
        coeff = arm_param.lift_motor['cail_step'] / arm_param.lift_motor['cail_dis']    #得到每个cm多少个脉冲
        target_pose = -target_cm * coeff                             #计算目标脉冲

        #调用服务
        res = self.__call_position_srv(self.__srv_lift_motor, cmd, target_pose, origin_param, ctrl_param)
        #脉冲转cm
        res.feedback.curr_point = res.feedback.curr_point / coeff
        return res

    def lift_motor_wait_finished(self):
        self.__logger.info('[arm] 升降电机 等待运动完成.')
        while True:
            time.sleep(0.1)
            res = self.lift_motor('read_feedback', 0, 0, False)
            #self.__logger.info(res)
            if res.feedback.reached:
                self.__logger.info("[arm] 升降运动完成.")
                break

    #调用positon ctrl服务
    def __call_position_srv(self, srv_client, cmd, target_pose, origin_param, ctrl_param):
        #命令字[读取反馈, 回原点, 设置位置, 关闭]
        cmd_num = {'read_feedback': 0, 'back_origin': 1, 'set_position': 2, 'disable': 3}
        req = CtrlImpl.Request()
        req.cmd = cmd_num[cmd]
        req.target_pose = target_pose
        req.origin_param.back_origin_vel = origin_param['back_origin_vel']
        req.origin_param.find_origin_cnt = origin_param['find_origin_cnt']
        req.origin_param.approix_origin_vel = origin_param['approix_origin_vel']
        req.origin_param.approix_origin_step = float(origin_param['approix_origin_step'])
        req.ctrl_param.kp = ctrl_param['kp']
        req.ctrl_param.ti = ctrl_param['ti']
        req.ctrl_param.td = ctrl_param['td']
        req.ctrl_param.max_vel = float(ctrl_param['max_vel'])
        req.ctrl_param.max_acc = float(ctrl_param['max_acc'])
        req.ctrl_param.low_pass = ctrl_param['low_pass']
        req.ctrl_param.ek = float(ctrl_param['ek'])
        req.ctrl_param.steady_clk = ctrl_param['steady_clk']


        res = srv_client.call(req)
        return res


    #创建控制参数
    def __create_axis_param(self, kp, ti, td, max_vel, max_acc, low_pass=0.5, ek=0.1, steady_clk=20):
        msg = {
            'kp': kp,
            'ti': ti,
            'td': td,
            'max_vel': max_vel,
            'max_acc': max_acc,
            'low_pass': low_pass,
            'ek': ek,
            'steady_clk': steady_clk
        }
        return msg

    #创建回原点参数
    def __create_origin_param(self, back_vel, find_cnt, approix_vel, approix_step):
        msg = {
            'back_origin_vel': back_vel,        #找原点速度(rpm)
            'find_origin_cnt': find_cnt,        #回原点次数(rpm)
            'approix_origin_vel': approix_vel,  #逼近速度(rpm)
            'approix_origin_step': approix_step #逼近步长(脉冲)
        }
        return msg