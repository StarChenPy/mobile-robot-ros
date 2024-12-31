import math
import time
import sys
sys.path.append('../')
from geometry_msgs.msg import Twist
from nav_msgs.srv import SetMap
from chassis_msgs.srv import CtrlMode,ResetOdom
"""
    底盘接口
    
    1: 切换手动自动模式
    2: 发布手动/自动线速度
    3: odometry接口 获取/设置
    4: 电量获取接口
    5: EMG状态获取接口
    
2024-03-13 com.huibo.robot Yongjie.Hu
"""

class ChassisImpl:
    __ros = None
    __srv_ctrl_mode = None          #控制模式srv 手动/自动
    __cmd_vel_pub = None
    __manual_vel_pub = None
    __srv_reset_odom = None

    __web_trans = None

    def __init__(self, ros, web_trans):
        self.__ros = ros
        self.__web_trans = web_trans

        self.__srv_ctrl_mode = self.__ros.create_client(CtrlMode, '/chassis/ctrl_mode') #, 'chassis_msgs/CtrlMode'
        self.__cmd_vel_pub = self.__ros.create_publisher(Twist, '/chassis/cmd_vel',10)
        self.__manual_vel_pub = self.__ros.create_publisher(Twist, '/chassis/manual_vel',10)
        self.__srv_reset_odom = self.__ros.create_client(ResetOdom, '/chassis/reset_odom') #, 'chassis_msgs/ResetOdom'
                
        print('[底盘接口] 初始化完成.')


    #设置为手动控制模式
    def set_manual_mode(self):
        print("[底盘接口] 设置手动模式")
        
        ctrl_req = CtrlMode.Request()
        ctrl_req.operation = 0 # 设置目标模式
        ctrl_req.mode = 1 # 设置手动模式
        
        res = self.__srv_ctrl_mode.call(ctrl_req)
        print(res)
        if(res.current_mode == ctrl_req.mode):
            print("[底盘接口] 设置手动模式成功")
        else:
            print("[底盘接口] 设置手动模式失败")

    #发布自动速度(x(m/s), y(m/s), w(rad/s))
    def set_cmdvel(self, x=0.0, y=0.0, theta=0.0):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(theta)
        
        self.__cmd_vel_pub.publish(msg)


    #发布手动速度(x(m/s), y(m/s), w(rad/s))
    def set_manual_vel(self, x=0.0, y=0.0, theta=0.0):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(theta)
        self.__manual_vel_pub.publish(msg)

    #设置odom mode:0重置[x y theta] 1重置[x y] 2重置[theta]
    def init_pose(self, x=0.0, y=0.0, w=0.0, mode=0):
        print("[底盘接口] 初始化机器人位置 [{} {} {}]".format(x, y, w))
        
        req = ResetOdom.Request()
        req.clear_mode = 0
        req.x = float(x)
        req.y = float(y)
        # W 角度转弧度
        if mode !=0 :
            pass
        else:
            w_ = math.radians(w)
            # w_ = w * math.pi / 180.0
            A_ = w_ * 180.0 / math.pi
            print(f'输入角度转弧度：{w_},弧度转回角度{A_}')
            req.theta = float(w_)
            
        res = self.__srv_reset_odom.call(req)
        
        if res.success == True:
            print("[底盘接口] 重置Odometry成功")
            return True
        else:
            print("[底盘接口] 重置Odometry错误")
            return False

    #获取电量
    def get_battery_vol(self):
        return self.__web_trans.read().battery
    #获取轮式odom
    def get_odometry(self):
        return self.__web_trans.read().odom
    #获取急停状态
    def get_emg_status(self):
        return self.__web_trans.read().emg
