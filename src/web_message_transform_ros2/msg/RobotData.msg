#传感器数据
float32[] sonar     #超声波距离
float32[] ir        #红外距离
float32[] lsb       #寻线传感器电压
Imu imu             #imu数据

#电池电压v
float32 battery

#odom坐标
Pose odom

#急停状态
bool emg

#robot pose
Pose robot_pose

#IO
bool[] di
vmxpi_msgs/TitanLimSW[] titan_limit_sw

#导航状态
int8 nav_status        #movebase导航状态 -1：未执行

