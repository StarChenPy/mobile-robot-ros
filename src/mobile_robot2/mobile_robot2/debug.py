import rclpy
from rclpy.node import Node


class Debug(Node):
    def __init__(self):
        super().__init__("debug")

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = Debug()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
