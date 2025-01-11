#titan驱动器设备信息
uint8 device_id         #设备id
uint8 major_version     #主版本号
uint8 minor_version     #次版本号
uint8 build_version     #编译版本号
uint8 hardware          #硬件(1 = Quad, 2 = Small)
uint8 hardware_version  #硬件版本
string unique_id        #ID
float32 temperature     #mcu温度
bool enable             #使能状态

