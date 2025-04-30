import rclpy
from rclpy.node import Node


class ShandongTrialsModule(Node):
    def __init__(self):
        super().__init__("shandong_trials_module")

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = ShandongTrialsModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
