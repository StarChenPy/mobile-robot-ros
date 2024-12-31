import time
from utils.impl.oms_param import *
import utils.impl.web_transform

from position_motor_ros2.srv import CtrlImpl

class OMSImpl:
    __ros = None
    __io_impl = None
    __srv_rotate_motor = None
    __srv_lift_motor = None

    def __init__(self, ros, web_trans, io_impl):
        self.__ros = ros
        self.__web_trans = web_trans
        self.__io_impl = io_impl

        self.__srv_rotate_motor = self.__ros.create_client(CtrlImpl,'/position_motor/rotate_motor/ctrl')
        self.__srv_lift_motor = self.__ros.create_client(CtrlImpl, '/position_motor/lift_motor/ctrl')

    #卡爪舵机 旋臂(角度)
    def gripper_rz(self, deg=0, release=False):
        print('[OMS 旋臂] 设置角度:{} 释放舵机:{}'.format(deg, release))
        
        if release:
            self.__io_impl.set_pwm(gripper_rz['pin'], 0.0)
            return
        
        #限位处理
        if deg < gripper_rz['min']:
            print('[OMS 旋臂] 警告 目标角度:{} 超出最小限位:{}'.format(deg, gripper_rz['min']))
            deg = gripper_rz['min']
        if deg > gripper_rz['max']:
            print('[OMS 旋臂] 警告 目标角度:{} 超出最大限位:{}'.format(deg, gripper_rz['max']))
            deg = gripper_rz['max']
        #角度转duty
        coeff = (gripper_rz['deg90_duty'] - gripper_rz['zero_duty']) / 90.0
        duty = gripper_rz['zero_duty'] + deg * coeff
        print(duty)
        self.__io_impl.set_pwm(gripper_rz['pin'], duty)

    #卡爪舵机 摆臂(角度)
    def gripper_ry(self, deg=0, release=False):
        print('[OMS 摆臂] 设置角度:{} 释放舵机:{}'.format(deg, release))
        
        if release:
            self.__io_impl.set_pwm(gripper_ry['pin'], 0.0)
            return
        
        #限位处理
        if deg < gripper_ry['min']:
            print('[OMS 摆臂] 警告 目标角度:{} 超出最小限位:{}'.format(deg, gripper_ry['min']))
            deg = gripper_ry['min']
        if deg > gripper_ry['max']:
            print('[OMS 摆臂] 警告 目标角度:{} 超出最大限位:{}'.format(deg, gripper_ry['max']))
            deg = gripper_ry['max']
        #角度转duty
        coeff = (gripper_ry['deg90_duty'] - gripper_ry['zero_duty']) / 90.0  # 9.0 - 6.0 / 90.0 = 0.033
        duty = gripper_ry['zero_duty'] + deg * coeff  # 6.0 + 60 * 0.033 
        #写入
        
        self.__io_impl.set_pwm(gripper_ry['pin'], duty)

    #伸缩(距离cm)
    def telescopic(self, distance=0, release=False):
        print('[OMS 伸缩] 设置距离:{}cm 释放舵机:{}'.format(distance, release))
        
        if release:
            self.__io_impl.set_pwm(telescopic_param['pin'], 0)
            return
        
        #限位处理
        if distance < 0:
            print('[OMS 伸缩] 警告 目标距离:{} 超出最小限位:{}'.format(distance, 0))
            distance = 0
        if distance > telescopic_param['itinerary']:
            print('[OMS 伸缩] 警告 目标距离:{} 超出最大限位:{}'.format(distance, telescopic_param['itinerary']))
            distance = telescopic_param['itinerary']
        #距离转duty  8.5 - 3.5 / 10   5
        coeff = (telescopic_param['max_duty'] - telescopic_param['min_duty']) / telescopic_param['itinerary'] # 0.5
        duty = telescopic_param['min_duty'] + distance * coeff
        
        duty = telescopic_param['max_duty'] - distance * coeff
        
        print(duty)

        self.__io_impl.set_pwm(telescopic_param['pin'], duty)

    #卡爪(距离cm)
    def gripper(self, distance=0, release=False):
        print('[OMS 卡爪] 设置距离:{}cm 释放舵机:{}'.format(distance, release))
        
        if release:
            self.__io_impl.set_pwm(gripper_param['pin'], 0)
            return
        
        #限位处理
        if distance < gripper_param['min_dis']:
            print('[OMS 卡爪] 警告 目标距离:{} 超出最小限位:{}'.format(distance, gripper_param['min_dis']))
            distance = gripper_param['min_dis']
        if distance > gripper_param['max_dis']:
            print('[OMS 卡爪] 警告 目标距离:{} 超出最大限位:{}'.format(distance, gripper_param['max_dis']))
            distance = gripper_param['max_dis']

        #距离转duty
        coeff = (gripper_param['max_duty'] - gripper_param['min_duty']) / gripper_param['itinerary'] # 10.0 - 3.5 / 10.5 
        duty = gripper_param['min_duty'] + distance / 2.0 * coeff   # 3.5 + 0.62 * dis /2
        print(duty)
        
        self.__io_impl.set_pwm(gripper_param['pin'], duty)

    #设置旋转电机命令[read_feedback, back_origin, set_position, disable]
    def rotate_motor(self, cmd='back_origin', target_deg=0, vel=100, log=True):
        if log:
            print('[oms] 旋转电机 命令:{} 目标角度:{}'.format(cmd, target_deg))
        #参数配置
        max_deg = rotate_motor['max_deg']      #最大角度
        min_deg = rotate_motor['min_deg']      #最小角度
        origin_param = self.__create_origin_param(rotate_motor['origin']['back_vel'], rotate_motor['origin']['find_cnt'], rotate_motor['origin']['approix_vel'],
                                                  rotate_motor['origin']['approix_step'])
        ctrl_param = self.__create_axis_param(rotate_motor['ctrl_param']['kp'], rotate_motor['ctrl_param']['ti'], rotate_motor['ctrl_param']['td'], vel, rotate_motor['ctrl_param']['max_acc'],
                                              rotate_motor['ctrl_param']['low_pass'], rotate_motor['ctrl_param']['ek'], rotate_motor['ctrl_param']['steady_clk'])
        
        # print("rotate_motor['ctrl_param']['ek']: {}".format(rotate_motor['ctrl_param']['ek']))
        #角度转脉冲
        if target_deg > max_deg:
            target_deg = max_deg
        if target_deg < min_deg:
            target_deg = min_deg
        enc_ppi = 1750.0 * 4.0
        # enc_ppi = 1464.0 * 4.0
        target_pose = target_deg * (enc_ppi / 360.0)
        #调用服务
        # print(ctrl_param)
        res = self.__call_position_srv(self.__srv_rotate_motor, cmd, target_pose, origin_param, ctrl_param)
        #反馈中的脉冲转角度
        res.feedback.curr_point = res.feedback.curr_point / (enc_ppi / 360.0)
        return res
       # print(res)

    #等待旋转电机完成
    def rotate_motor_wait_finished(self):
        print('[oms] 旋转电机 等待运动完成.')
        while True:
            time.sleep(0.1)
            res = self.rotate_motor('read_feedback', 0, 0, False)
            
            if res.feedback.reached == True:
                print("[oms] 旋转运动完成.")
                break

    def lift_motor(self, cmd='back_origin', target_cm=0, vel=100.0, log=True):
        if log:
            print('[oms] 升降电机 命令:{} 目标距离:{}cm'.format(cmd, target_cm))
        #参数配置
        origin_param = self.__create_origin_param(lift_motor['origin']['back_vel'], lift_motor['origin']['find_cnt'], lift_motor['origin']['approix_vel'],
                                                  lift_motor['origin']['approix_step'])
        ctrl_param = self.__create_axis_param(lift_motor['ctrl_param']['kp'], lift_motor['ctrl_param']['ti'], lift_motor['ctrl_param']['td'], vel, lift_motor['ctrl_param']['max_acc'],
                                              lift_motor['ctrl_param']['low_pass'], lift_motor['ctrl_param']['ek'], lift_motor['ctrl_param']['steady_clk'])

        #软限位
        if target_cm > lift_motor['max_dis']:
            target_cm = lift_motor['max_dis']
        if target_cm < lift_motor['min_dis']:
            target_cm = lift_motor['min_dis']

        #单位转换
        coeff = lift_motor['cail_step'] / lift_motor['cail_dis']    #得到每个cm多少个脉冲
        target_pose = -target_cm * coeff                             #计算目标脉冲

        #调用服务
        res = self.__call_position_srv(self.__srv_lift_motor, cmd, target_pose, origin_param, ctrl_param)
        #脉冲转cm
        res.feedback.curr_point = res.feedback.curr_point / coeff
        return res

    def lift_motor_wait_finished(self):
        print('[oms] 升降电机 等待运动完成.')
        while True:
            time.sleep(0.1)
            res = self.lift_motor('read_feedback', 0, 0, False)
            #print(res)
            if res.feedback.reached == True:
                print("[oms] 升降运动完成.")
                break

    #调用positon ctrl服务
    def __call_position_srv(self, srv_client, cmd, target_pose, origin_param, ctrl_param):
        #命令字[读取反馈, 回原点, 设置位置, 关闭]
        cmd_num = {'read_feedback': 0, 'back_origin': 1, 'set_position': 2, 'disable': 3}
        # msg = {
        #     'cmd': cmd_num[cmd],
        #     'target_pose': target_pose,
        #     'origin_param': origin_param,
        #     'ctrl_param': ctrl_param
        # }
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
        # req = roslibpy.ServiceRequest(msg)
        # res = srv_client.call(req)
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

