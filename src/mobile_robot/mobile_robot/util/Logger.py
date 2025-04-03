import inspect
from typing import Optional

from rclpy.impl.logging_severity import LoggingSeverity

from .Singleton import singleton


@singleton
class Logger:
    def __init__(self):
        self.ros_logger = None

    def set_ros_logger(self, logger, level=LoggingSeverity.INFO):
        """
        设置ROS2日志管理器
        @param logger: ROS2的Logger对象
        @param level: 日志等级
        """
        self.ros_logger = logger
        self.ros_logger.set_level(level)

    def debug(self, msg: str) -> None:
        """记录DEBUG级别日志"""
        self._log('debug', msg)

    def info(self, msg: str) -> None:
        """记录INFO级别日志"""
        self._log('info', msg)

    def warn(self, msg: str) -> None:
        """记录WARN级别日志"""
        self._log('warning', msg)

    def error(self, msg: str) -> None:
        """记录ERROR级别日志"""
        self._log('error', msg)

    def _log(self, level: str, msg: str) -> None:
        """统一的日志记录方法"""
        context = self._get_caller_context()
        formatted_msg = f"[{context}] {msg}"
        getattr(self.ros_logger, level)(formatted_msg)

    def _get_caller_context(self) -> str:
        """
        获取调用者的上下文信息
        返回格式：ClassName::method_name 或 ModuleName::function_name
        """
        try:
            # 跳过当前栈帧和直接调用栈帧（_log方法）
            frame = inspect.currentframe().f_back.f_back
            caller_frame_info = inspect.getframeinfo(frame)

            # 获取方法/函数名
            method_name = caller_frame_info.function

            # 尝试获取类名
            instance = self._get_frame_instance(frame)
            if instance:
                return f"{instance.__class__.__name__}::{method_name}"

            # 尝试获取模块名
            module = inspect.getmodule(frame)
            if module:
                return f"{module.__name__}::{method_name}"

            return f"unknown::{method_name}"
        finally:
            # 显式释放frame引用以避免内存泄漏
            del frame

    @staticmethod
    def _get_frame_instance(frame) -> Optional[object]:
        """从frame中获取类实例（如果存在）"""
        return frame.f_locals.get('self', None)