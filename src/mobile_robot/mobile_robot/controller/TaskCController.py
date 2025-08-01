import time

from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TaskCController:
    def __init__(self, node):
        self.logger = Logger()
        self.node = node

    def run(self):
        start_time = time.time()

        end_time = time.time()
        use_time = end_time - start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")
