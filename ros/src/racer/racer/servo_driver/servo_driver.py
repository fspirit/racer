from rclpy.node import Node

from racer_msgs.msg import Actuators

class ServoDriver(Node):

    throttle_gain = 0.7
    throttle_offset = 0.1

    steering_gain = 0.545
    steering_offset = 0.205

    def __init__(self, steering_motor, throttle_motor):
        super().__init__('servo_driver')

        self.steering_motor = steering_motor
        self.throttle_motor = throttle_motor

        self.subscription = self.create_subscription(
            Actuators,
            'actuators',
            self.handle_command,
            10)

    def convert_throttle(self,  throttle_value):
        return throttle_value * self.throttle_gain + self.throttle_offset

    def convert_steering(self,  steering_value):
        return steering_value * self.steering_gain + self.steering_offset

    def handle_command(self, msg):
        self.steering_motor.throttle = self.convert_steering(msg.data.steering)
        self.throttle_motor.throttle = self.convert_throttle(msg.data.throttle)