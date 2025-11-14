#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'motor_cmd', 10)
        self.get_logger().info("Keyboard control active — use W/A/S/D keys to move, Q to quit")
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            msg = Twist()

            # WASD key bindings
            if key == 'w':
                msg.linear.y = 1.0
            elif key == 's':
                msg.linear.y = -1.0
            elif key == 'a':
                msg.linear.x = -1.0
            elif key == 'd':
                msg.linear.x = 1.0
            elif key == 'q':
                self.get_logger().info("Exiting keyboard control.")
                break
            else:
                continue

            # Publish Twist message
            self.publisher.publish(msg)
            self.get_logger().info(f"Sent command: X={msg.linear.x}, Y={msg.linear.y}")

def main(args=None):
    rclpy.init(args=args)
    KeyboardController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
