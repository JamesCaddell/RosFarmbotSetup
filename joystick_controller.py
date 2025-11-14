#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.publisher = self.create_publisher(Twist, 'motor_cmd', 10)
        self.get_logger().info("Joystick controller node started")

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected. Plug one in and restart this node.")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Using joystick: {self.joystick.get_name()}")

        # Control parameters
        self.deadzone = 0.1
        self.rate = self.create_rate(20)  # 20 Hz update rate

        self.run()

    def run(self):
        while rclpy.ok():
            pygame.event.pump()  # Process pygame events

            # Read axis data (commonly 0 = left stick X, 1 = left stick Y)
            x_axis = self.joystick.get_axis(0)
            y_axis = -self.joystick.get_axis(1)  # invert Y for intuitive control

            # Deadzone filtering
            if abs(x_axis) < self.deadzone:
                x_axis = 0.0
            if abs(y_axis) < self.deadzone:
                y_axis = 0.0

            # Create and publish Twist message
            msg = Twist()
            msg.linear.x = x_axis
            msg.linear.y = y_axis
            self.publisher.publish(msg)

            # Log values (optional)
            self.get_logger().info(f"Joystick X={x_axis:.2f}, Y={y_axis:.2f}")

            time.sleep(0.05)  # ~20Hz loop

def main(args=None):
    rclpy.init(args=args)
    JoystickController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
