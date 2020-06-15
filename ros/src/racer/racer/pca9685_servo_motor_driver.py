import rclpy

from rclpy.node import Node
from adafruit_servokit import ServoKit
from racer_msgs.msg import Actuators

class ServoPca9685Driver(Node):

    # Physical connection setup
    i2c_address = 0x40
    steering_channel = 0
    throttle_channel = 1

    def __init__(self):
        super().__init__('servo_pca9685_driver')

        self.kit = ServoKit(channels=16, address=self.i2c_address)

        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]

        self.subscription = self.create_subscription(
            Actuators,
            'actuators',
            self.handle_command,
            10)

    def convert_throttle(self,  throttle_value):
        # Throttle calibration
        throttle_gain = 0.7
        throttle_offset = 0.1

        return throttle_value * throttle_gain + throttle_offset

    def convert_steering(self,  steering_value):
        # Steering calibration
        steering_gain = 0.545
        steering_offset = 0.205

        return steering_value * steering_gain + steering_offset

    def handle_command(self, msg):
        self.get_logger().info("Received Actuators: throttle = {}, steering = {}".format(
            msg.throttle, msg.steering))
        self.get_logger().info("Converted to: throttle = {}, steering = {}".format(
            self.convert_throttle(msg.throttle), self.convert_steering(msg.steering)))  

        self.steering_motor.throttle = self.convert_steering(msg.steering)
        self.throttle_motor.throttle = self.convert_throttle(msg.throttle)

def main(args=None):
    rclpy.init(args=args)

    servo_driver = ServoPca9685Driver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()