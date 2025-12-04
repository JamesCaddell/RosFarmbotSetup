#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# --- Pin setup for Y motor ---
DIR_PIN = 24    # Direction pin for Y motor (BCM 24)
STEP_PIN = 25   # Step pin for Y motor     (BCM 25)
STEP_DELAY = 0.001  # Adjust if needed (0.0005–0.002 is typical)

class MotorY(Node):
    def __init__(self):
        super().__init__('motor_y')

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(STEP_PIN, GPIO.OUT)

        # Subscribe to the same topic as X motor
        self.subscription = self.create_subscription(
            Twist,
            'motor_cmd',
            self.listener_callback,
            10
        )

        self.get_logger().info('Motor Y node ready. Listening on /motor_cmd')

    def listener_callback(self, msg):
        if msg.linear.y == 0:
            return

        # Direction: positive = up/forward, negative = down/backward
        direction = GPIO.HIGH if msg.linear.y > 0 else GPIO.LOW
        GPIO.output(DIR_PIN, direction)

        # 1.0 in Twist → 200 steps (same scaling as X for consistency)
        steps = int(abs(msg.linear.y) * 200)

        self.get_logger().info(f'Moving Y motor {"+" if msg.linear.y > 0 else "-"} for {steps} steps')

        for _ in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEP_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEP_DELAY)


def main(args=None):
    rclpy.init(args=args)
    node = MotorY()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
