class IOImpl:
    def __init__(self, ros, web_trans):
        self.__ros = ros
        self.__web_trans = web_trans

    #读取按键
    def read_button(self):
        return self.read_di(1)

    #设置LED灯
    def set_led(self, state=False):
        self.set_do(0, state)


    #设置DO输出(端口:0-2, 电平:T/F)
    def set_do(self, port=0, status=False):
        self.__web_trans.write_do(port, status)

    #设置pwm(端口:0-4, duty:0-100%)
    def set_pwm(self, port=0, duty=0.0):
        self.__web_trans.write_pwm(port, float(duty))

    #读取DI输入(端口:0-1)
    def read_di(self, port=0):
        return self.__web_trans.read().di[port]

    #读取titan限位开关输入
    def read_titan_sw(self, port=0):
        return self.__web_trans.read().titan_limit_sw[port]