
import math
import time
from rclpy.action import ActionClient
from base_nav.action import NavigationActionCMD
from geometry_msgs.msg import Twist
from nav_msgs.srv import SetMap
from base_motion_ros2.srv import BaseMotion
# from robot_manager.msg import PathPointImpl
# from robot_manager.msg import PoseInit
"""
        导航接口
    1: 导航到坐标
    2: 导航到路径点
    3: 多路径点跟随导航
    4: 取消导航
    5: 等待导航结束
    6: 路径点数据获取
    7: amcl pose获取
    8: 初始定位amcl设置
    9: 导航状态

2024-03-12 com.huibo.robot Yongjie.Hu
"""
class NavigationImpl:
    is_nav_result_ = 0
    __ros = None
    __web_trans = None
    __action_movebase = None          #movebase导航action
    __goal = None                     #move_base goal
    __srv_base_motion = None            #amcl pose init srv
    __is_cancel = False
    __is_nav_result = 0
    __nav_result_message = None


    def __init__(self, ros, web_trans):
        self.__ros = ros
        self.__web_trans = web_trans
        #路径点接口服务
        self.__srv_base_motion = self.__ros.create_client(BaseMotion, '/base_motion') # 'robot_manager/PathPointImpl'        
        #movebase action
        self.__action_movebase = ActionClient(self.__ros, NavigationActionCMD, '/nav_action') #, 'move_base_msgs/MoveBaseAction'
        #topic move_base/cancel
        # self.__cancel_pub = self.__ros.create_publisher('/move_base/cancel', 'actionlib_msgs/GoalID')
        # amcl pose init srv
        # self.__srv_init_pose = self.__ros.create_client(SetMap, '/robot_manager/pose_init') #, 'robot_manager/PoseInit'
        #获取路径点 FIX 应该不需要
        # self.__path_points = self.__call_srv_path_point()
        print('[导航接口] 初始化完成.')


    #导航到pose
    def move_to_point(self, x=0.0, y=0.0, w=0.0, is_rotate=False, max_vel = 0.2):
        #w转四元数
        theta = w * math.pi / 180.0
        qz = math.sin(theta * 0.5)
        qw = math.cos(theta * 0.5)
        
        goal_msg = NavigationActionCMD.Goal()
        goal_msg.target_x = float (x)
        goal_msg.target_y = float (y)
        goal_msg.target_theta = float (w)
        goal_msg.is_rotation = is_rotate
        goal_msg.param.max_vel = max_vel
        
        self.__action_movebase.wait_for_server()
        
        print("[导航接口] 正在发送新的导航请求[{} {} {}]".format(x, y, w))
        
        self.__is_nav_result = -1 # 开启导航
        self.is_nav_result_ = -1

		# 创建action的句柄
        self.send_goal_future = self.__action_movebase.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
                
        return True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            # self.get_logger().info('拒绝目标点 :(')
            return
        
        # self.get_logger().info('正在执行目标点 :)')
        
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        nav_result = future.result().result
        self.__is_nav_result = 1
        self.is_nav_result_ = 1
        self.__nav_result_message = nav_result.message
            
        print("[导航接口] 执行完毕: ",nav_result.message)

    def point_follow(self, points=[], max_vel = 0.2):
        # print(*points, sep='\n')
        self.__is_cancel = False
        pose = []
        #获取整个路径pose[x y w]
        for pt in points:
            pose.append({'x': pt['x'], 'y': pt['y'], 'w': pt['w']})

        #导航
        i = 0
        for pt in pose:
            print("[导航接口] 正在导航至目标点:", points[i])
            
            is_rotate = False
            # 最后一个点需要主动执行最后的yaw
            if(i == pose.__len__() - 1):
                is_rotate = True
                
            self.move_to_point(pt['x'], pt['y'], pt['w'], is_rotate, max_vel)
            self.wait_finished()
            i = i + 1
            if self.__is_cancel:
                return True
        return True

    #等待导航结束 并且返回打印string值
    def wait_finished(self):
        print("[导航接口] 等待导航结束中...")
    
        while self.__is_nav_result != 1:
            time.sleep(0.1)
        print("[导航接口] 导航结束...")
            
        return self.__nav_result_message

    #获取导航状态 -1:未执行过导航
    def get_nav_status(self):
        return self.__web_trans.read().nav_status

    #取消导航
    def cancel(self):
        cancel_future = self.goal_handle.cancel_goal_async()        

    # #初始化pose amcl
    # def init_pose(self, x=0, y=0, w=0):
    #     msg = {
    #         "pose": {
    #             "x": x,
    #             "y": y,
    #             "w": w
    #         }
    #     }
    #     print("[导航接口] 正在初始化AMCL位姿 [{} {} {}]".format(x, y, w))
    #     req = roslibpy.ServiceRequest(msg)
    #     res = self.__srv_init_pose .call(req)

    #     if res["success"] == True:
    #         print("[导航接口] AMCL初始位姿完成")
    #         return True
    #     else:
    #         print("[导航接口] AMCL初始位姿错误")
    #         return False

    def get_robot_pose(self):
        pose = self.__web_trans.read().robot_pose
        return {'x': pose.x, 'y': pose.y, 'w': pose.w}
        return self.__web_trans.read().robot_pose
    
    def __call_srv_base_motion(self, motion_mode, set_point, max_vel):
        #直线参数
        # line_param = self.__create_axis_param(1.5, 0.0, 0.0, max_vel, 3.0, 0.8, 0.01, 5)

        req = BaseMotion.Request()
        req.motion_mode = motion_mode
        req.set_point = float(set_point)
        req.line_param.kp = 1.5
        req.line_param.ti = 0.0
        req.line_param.td = 0.0
        req.line_param.max_vel = float(max_vel)
        req.line_param.max_acc = 3.0
        req.line_param.low_pass = 0.8
        req.line_param.ek = 0.01
        req.line_param.steady_clk = 5


        # rotate_param = {}
        if motion_mode == 2:
            # 参数 直线运动模式
            req.rotate_param.kp = 2.3
            req.rotate_param.ti = 0.0
            req.rotate_param.td = 0.0001
            req.rotate_param.max_vel = float(max_vel)
            req.rotate_param.max_acc = 600.0
            req.rotate_param.low_pass = 0.8
            req.rotate_param.ek = 5.0
            req.rotate_param.steady_clk = 5
        
            # rotate_param = self.__create_axis_param(2.3, 0.0, 0.0001, max_vel, 600, 0.8, 5.0, 5)
        elif motion_mode == 3:
            #参数 旋转运动模式
            req.rotate_param.kp = 1.8
            req.rotate_param.ti = 0.0
            req.rotate_param.td = 0.0001
            req.rotate_param.max_vel = float(max_vel)
            req.rotate_param.max_acc = 600.0
            req.rotate_param.low_pass = 0.7
            req.rotate_param.ek = 1.0
            req.rotate_param.steady_clk = 10
            # rotate_param = self.__create_axis_param(1.8, 0.0, 0.0001, max_vel, 600, 0.7, 1.0, 10)

        # msg = {
        #     'motion_mode': motion_mode,
        #     'set_point': set_point,
        #     'line_param': line_param,
        #     'rotate_param': rotate_param
        # }   
        # req = roslibpy.ServiceRequest(msg)
        res = self.__srv_base_motion.call(req)
        return res    
    
    
    #base motion 旋转
    def base_motion(self, mode='rotate', set_deg=0.0, max_vel=0.0):
        print('[基础运动] {}. 设定值:{} 速度:{}'.format(mode, set_deg, max_vel))
        mode_tbl = {'line': 2, 'rotate': 3}
        res = self.__call_srv_base_motion(mode_tbl[mode], set_deg, max_vel)
        success = res.success
        if not success:
            print('[基础运动] 错误, 无法启动运动!', mode)
        return success

    #停止基础运动
    def stop_base_motion(self):
        print("start")
        res = self.__call_srv_base_motion(1, 0, 0)
        if res.success:
            print('[基础运动] 停止完成.')
        else:
            print('[基础运动] 停止失败.')
        return res.success

    #获取基础运动反馈
    def base_motion_feedback(self):
        res = self.__call_srv_base_motion(0, 0, 0)
        return res
    
    #等待基础运动完成
    def wait_base_motion(self):
        """
           {'feedback': {'motion_mode': 0, 'motion_status': 2, 'error': 0.0}, 'success': True}
            motion_mode: 0:空闲 1:直线 2:旋转
            motion_status: 0:空闲 1:运行中 2:完成
        """
        while True:
            res = self.__call_srv_base_motion(0, 0, 0)
            #print(res)
            if res.feedback.motion_mode == 0 or res.feedback.motion_status == 2 :
                break
            time.sleep(0.1)
