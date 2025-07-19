import rclpy


class MotionDebuggerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("motion_debugger")


def main():
    rclpy.init()

    node = MotionDebuggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
