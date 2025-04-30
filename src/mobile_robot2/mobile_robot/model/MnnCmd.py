import enum


class MnnCmd(enum.Enum):
    CMD_QUERY = 0                 #查询命令
    CMD_LOAD_MODEL  = 1           #加载模型
    CMD_CLOSE_MODEL = 2           #关闭模型
    CMD_DETECT_ONCE = 3           #检测一次
    CMD_GET_RESULT_IMG = 4        #获取检测结果图片