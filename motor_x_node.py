import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorXController(Node):
    def __init__(self):
        super().__init__('motor_x_controller')
        self.subscription = self.create_subscription(
            Float32,
            '/motor_x_speed',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info('Motor X Controller Started')

    def listener_callback(self, msg):
        speed = msg.data
        self.get_logger().info(f'Motor X Speed Command: {speed:.2f}')
        # GPIO or hardware control logic would go here

def main(args=None):
    rclpy.init(args=args)
    node = MotorXController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
