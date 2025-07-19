import rclpy


class MotionActuatorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("motion_actuator")


def main():
    rclpy.init()

    node = MotionActuatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
