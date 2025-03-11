#轴闭环控制参数
#PID参数
float32 kp
float32 ti
float32 td
#滤波器参数
float32 max_vel     #最大速度
float32 max_acc     #最大加速度
float32 low_pass    #低通平滑系数[0-1]
#跳出条件
float32 ek          #误差因子
uint8 steady_clk    #稳态周期数 1clk=20ms
#规划器参数
float32 planner_vel     #速度
float32 planner_acc     #加速度
float32 planner_decel   #减加速度