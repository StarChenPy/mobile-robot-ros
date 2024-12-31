import rclpy

from impl.robot_correct import Controls


class VmxRobot(rclpy.Node):
    def __init__(self):
        super().__init__('vmx_robot')

        self.controls_cmd = Controls(self)
        self.create_timer(2, self.mnn)

    def mnn(self):
        self.controls_cmd.send_mnn_request(2,'')
        self.controls_cmd.mnn_result()


def main():
    rclpy.init()

    robot = VmxRobot()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()