#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO # type: ignore

class L298NOmniDriver(Node):
    def __init__(self):
        super().__init__('l298n_omni_driver')

        # Motor 1 pins (e.g. front-left wheel)
        self.m1_in1 = 17
        self.m1_in2 = 27
        self.m1_pwm_pin = 22

        # Motor 2 pins (e.g. front-right wheel)
        self.m2_in1 = 23
        self.m2_in2 = 24
        self.m2_pwm_pin = 25

        # Motor 3 pins (e.g. rear wheel)
        self.m3_in1 = 5
        self.m3_in2 = 6
        self.m3_pwm_pin = 13

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        motor_pins = [
            self.m1_in1, self.m1_in2, self.m1_pwm_pin,
            self.m2_in1, self.m2_in2, self.m2_pwm_pin,
            self.m3_in1, self.m3_in2, self.m3_pwm_pin,
        ]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        # PWM setup
        self.m1_pwm = GPIO.PWM(self.m1_pwm_pin, 100)
        self.m2_pwm = GPIO.PWM(self.m2_pwm_pin, 100)
        self.m3_pwm = GPIO.PWM(self.m3_pwm_pin, 100)

        self.m1_pwm.start(0)
        self.m2_pwm.start(0)
        self.m3_pwm.start(0)

        self.subscription = self.create_subscription(
            String,
            'cmd_omni',
            self.command_callback,
            10
        )

        self.get_logger().info("L298N 3-Omni motor driver started")

    def set_motor(self, in1, in2, pwm, direction, speed):
        if direction == 'forward':
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            # Stop motor
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            speed = 0
        pwm.ChangeDutyCycle(speed)

    def stop_all(self):
        self.set_motor(self.m1_in1, self.m1_in2, self.m1_pwm, 'stop', 0)
        self.set_motor(self.m2_in1, self.m2_in2, self.m2_pwm, 'stop', 0)
        self.set_motor(self.m3_in1, self.m3_in2, self.m3_pwm, 'stop', 0)

    def command_callback(self, msg):
        cmd = msg.data.lower()
        self.get_logger().info(f"Command received: {cmd}")

        speed = 70  # default speed %

        if cmd == 'forward':
            # Example: all wheels forward
            self.set_motor(self.m1_in1, self.m1_in2, self.m1_pwm, 'forward', speed)
            self.set_motor(self.m2_in1, self.m2_in2, self.m2_pwm, 'forward', speed)
            self.set_motor(self.m3_in1, self.m3_in2, self.m3_pwm, 'forward', speed)

        elif cmd == 'backward':
            self.set_motor(self.m1_in1, self.m1_in2, self.m1_pwm, 'backward', speed)
            self.set_motor(self.m2_in1, self.m2_in2, self.m2_pwm, 'backward', speed)
            self.set_motor(self.m3_in1, self.m3_in2, self.m3_pwm, 'backward', speed)

        elif cmd == 'left':
            # Rotate or strafe logic depending on your omni setup
            # Example strafing left:
            self.set_motor(self.m1_in1, self.m1_in2, self.m1_pwm, 'backward', speed)
            self.set_motor(self.m2_in1, self.m2_in2, self.m2_pwm, 'forward', speed)
            self.set_motor(self.m3_in1, self.m3_in2, self.m3_pwm, 'stop', 0)

        elif cmd == 'right':
            self.set_motor(self.m1_in1, self.m1_in2, self.m1_pwm, 'forward', speed)
            self.set_motor(self.m2_in1, self.m2_in2, self.m2_pwm, 'backward', speed)
            self.set_motor(self.m3_in1, self.m3_in2, self.m3_pwm, 'stop', 0)

        elif cmd == 'stop':
            self.stop_all()

        else:
            self.get_logger().warning("Unknown command")

    def destroy_node(self):
        self.stop_all()
        self.m1_pwm.stop()
        self.m2_pwm.stop()
        self.m3_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = L298NOmniDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
