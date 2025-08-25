import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10 
        )

        self.publisher = self.create_publisher(
            Twist,
            '/chassis/cmd_vel',
            10  
        )
        
        self.get_logger().info('正在转发 /cmd_vel 到 /chassis/cmd_vel')

    def cmd_vel_callback(self, msg):
        self.publisher.publish(msg)

def main():
    rclpy.init()

    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
