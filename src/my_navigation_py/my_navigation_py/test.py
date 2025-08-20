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
        
        self.get_logger().info('CmdVelBridge节点已启动，正在转发/cmd_vel到/chassis/cmd_vel...')

    def cmd_vel_callback(self, msg):

        self.publisher.publish(msg)
        # self.get_logger().info(f"转发速度指令: 线速度={msg.linear.x}, 角速度={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_bridge = CmdVelBridge()
    
    try:
        rclpy.spin(cmd_vel_bridge)
    except KeyboardInterrupt:
        cmd_vel_bridge.get_logger().info('用户中断，正在关闭节点...')
    finally:
        cmd_vel_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    