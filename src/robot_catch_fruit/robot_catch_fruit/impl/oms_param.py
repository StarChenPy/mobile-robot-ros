#############################舵机参数############################
#卡爪绕Y轴旋转舵机 摆臂  fixed
gripper_ry = {
    'pin': 1,               #端口pwm(?)
    'min': -95,               #最小角度
    'max': 90,              #最大角度
    'zero_duty': 28.0,       #卡爪零位时占空比
    'deg90_duty': 11 # 8.8       #卡爪90度时占空比
}
#     'deg90_duty': 7 # 8.8       #卡爪90度时占空比

#卡爪绕Z轴旋转舵机 旋臂  fixed
gripper_rz = {
    'pin': 4,  # 端口pwm(?)
    'min': -40,  # 最小角度
    'max': 90,  # 最大角度
    'zero_duty': 23.55,  # 卡爪零位时占空比
    'deg90_duty': 12.34  # 卡爪90度时占空比
}

#伸缩舵机参数  fixed
telescopic_param = {
    'pin': 2,                 #端口pwm(?)
    'min_duty': 12,  #3.5,          #最小占空比
    'max_duty': 49, #8.5,          #最大占空比
    'itinerary': 15 # 22.0         #行程cm
}

#卡爪舵机参数 
gripper_param = {
    'pin': 3,                 #端口pwm(?)
    'min_dis': 0,               #最小距离cm
    'max_dis': 24.5,            #最大距离cm
    'min_duty': 11.0,          #最小占空比
    'max_duty': 41.0,          #最大占空比
    'itinerary': 10.5         #行程cm
}

############################旋转电机参数##########################
rotate_motor = {
    'max_deg': 180.0,                #最大旋转角度限位
    'min_deg': -90.0,               #最小旋转角度限位

    #回原点参数
    'origin':{
        'back_vel': 10.0,          #找原点速度(rpm)
        'find_cnt': 2,              #找原点次数
        'approix_vel': 5.0,        #逼近原点速度(rpm)
        'approix_step': 50.0          #逼近原点步长(脉冲)
    },

    #位环控制参数
    'ctrl_param': {
        'kp': 0.10,                 #kp
        'ti': 0.0,                  #ti
        'td': 0.0,                  #td
        'max_acc': 2000.0,          #最大加速度(脉冲/s^2)
        'low_pass': 0.8,            #速度低通滤波系数[0,1]
        'ek': 5,                    #误差因子(脉冲)
        'steady_clk': 10            #稳态周期(1clk=20ms)
    }
}

############################升级电机参数##########################
lift_motor = {
    'max_dis': 28.0,                 #最大升降限位cm
    'min_dis': 0.0,                  #最小升降限位cm

    #单位标定参数
    'cail_step': 3000.0,            #脉冲  fixed
    'cail_dis': 30, #17.0,               #对应距离cm  fixed

    #回原点参数
    'origin':{
        'back_vel': 100.0, #-50.0,          #找原点速度(rpm)  fixed
        # 'back_vel': 10.0, #-50.0,          #找原点速度(rpm)  fixed
        'find_cnt': 2,              #找原点次数
        'approix_vel': 30.0, #-30.0,       #逼近原点速度(rpm)  fixed
        # 'approix_vel': 5.0, #-30.0,       #逼近原点速度(rpm)  fixed
        'approix_step': 300         #逼近原点步长(脉冲)
    },

    #位环控制参数
    'ctrl_param': {
        'kp': 0.05,                  #kp
        'ti': 0.0,                  #ti
        'td': 0.0,                  #td
        'max_acc': 2000.0,          #最大加速度(脉冲/s^2)
        'low_pass': 0.8,            #速度低通滤波系数[0,1]
        'ek': 10,                   #误差因子(脉冲)
        'steady_clk': 10            #稳态周期(1clk=20ms)
    }
}
