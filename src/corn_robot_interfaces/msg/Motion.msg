# 动作名称
string name

# 机械臂参数
float32 rotate
float32 lift
float32 rotary_servo
float32 nod_servo
float32 telescopic_servo
float32 gripper_servo

# 子节点与对应的条件
ConnectedNode[] connected_nodes

# 本节点执行后的状态更新
KeyBool[] update_status
