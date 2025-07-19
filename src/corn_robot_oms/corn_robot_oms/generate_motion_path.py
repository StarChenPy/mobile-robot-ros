import rclpy


class GenerateMotionPathNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("generate_motion_path")


def main():
    rclpy.init()

    node = GenerateMotionPathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
