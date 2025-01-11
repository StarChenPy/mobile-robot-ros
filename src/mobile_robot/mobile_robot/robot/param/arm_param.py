############################# 舵机参数配置 #############################

# 定义舵机参数的基础模板
def create_servo_param(pin, min_angle, max_angle, zero_duty=None, deg90_duty=None, min_duty=None, max_duty=None, itinerary=None):
    return {
        'pin': pin,                # PWM端口
        'min': min_angle,          # 最小角度或最小物理量
        'max': max_angle,          # 最大角度或最大物理量
        'zero_duty': zero_duty,    # 零位时的占空比
        'deg90_duty': deg90_duty,  # 90度时的占空比（适用角度控制）
        'min_duty': min_duty,      # 最小占空比（适用物理量控制）
        'max_duty': max_duty,      # 最大占空比（适用物理量控制）
        'itinerary': itinerary     # 行程（cm）
    }

# 舵机配置
gripper_ry = create_servo_param(pin=1, min_angle=-95, max_angle=90, zero_duty=28.0, deg90_duty=11)
gripper_rz = create_servo_param(pin=4, min_angle=-40, max_angle=90, zero_duty=23.55, deg90_duty=12.34)
telescopic_param = create_servo_param(pin=2, min_angle=0, max_angle=15, min_duty=12, max_duty=49, itinerary=15)
gripper_param = create_servo_param(pin=3, min_angle=0, max_angle=24.5, min_duty=11.0, max_duty=41.0, itinerary=10.5)

############################# 电机参数配置 #############################

# 定义电机参数的基础模板
def create_motor_param(max_value, min_value, origin_config, ctrl_param, cail_step=None, cail_dis=None):
    return {
        'max_value': max_value,         # 最大物理量（角度或距离）
        'min_value': min_value,         # 最小物理量（角度或距离）
        'origin': origin_config,        # 回原点参数
        'ctrl_param': ctrl_param,       # 位环控制参数
        'cail_step': cail_step,         # 脉冲数（标定用）
        'cail_dis': cail_dis            # 对应距离（标定用）
    }

# 定义回原点参数模板
def create_origin_config(back_vel, find_cnt, approix_vel, approix_step):
    return {
        'back_vel': back_vel,           # 找原点速度（rpm）
        'find_cnt': find_cnt,           # 找原点次数
        'approix_vel': approix_vel,     # 逼近原点速度（rpm）
        'approix_step': approix_step    # 逼近原点步长（脉冲）
    }

# 定义位环控制参数模板
def create_ctrl_param(kp, ti, td, max_acc, low_pass, ek, steady_clk):
    return {
        'kp': kp,                       # 比例系数
        'ti': ti,                       # 积分时间
        'td': td,                       # 微分时间
        'max_acc': max_acc,             # 最大加速度（脉冲/s^2）
        'low_pass': low_pass,           # 速度低通滤波系数
        'ek': ek,                       # 误差因子（脉冲）
        'steady_clk': steady_clk        # 稳态周期（1clk=20ms）
    }

# 旋转电机配置
rotate_motor = create_motor_param(
    max_value=180.0,
    min_value=-90.0,
    origin_config=create_origin_config(back_vel=10.0, find_cnt=2, approix_vel=5.0, approix_step=50.0),
    ctrl_param=create_ctrl_param(kp=0.10, ti=0.0, td=0.0, max_acc=2000.0, low_pass=0.8, ek=5, steady_clk=10)
)

# 升降电机配置
lift_motor = create_motor_param(
    max_value=28.0,
    min_value=0.0,
    origin_config=create_origin_config(back_vel=100.0, find_cnt=2, approix_vel=30.0, approix_step=300),
    ctrl_param=create_ctrl_param(kp=0.05, ti=0.0, td=0.0, max_acc=2000.0, low_pass=0.8, ek=10, steady_clk=10),
    cail_step=3000.0,
    cail_dis=30.0
)
