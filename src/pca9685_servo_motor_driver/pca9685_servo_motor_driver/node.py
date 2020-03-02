import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import String

class ServoPca9685Driver(rclpy.Node):

    # Physical connection setup
    i2c_address = 0x40
    steering_channel = 0
    throttle_channel = 1
    # Steering calibration
    steering_gain = 0.545
    steering_offset = 0.205
    # Throttle calibration
    throttle_gain = 0.7
    throttle_offset = 0.1

    def __init__(self):
        super().__init__('servo_pca9685_driver')

        self.kit = ServoKit(channels=16, address=self.i2c_address)

        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]

        self.subscription = self.create_subscription(
            String,
            'actuators',
            self.handle_command,
            10)

    def handle_command(self, msg):
        self.steering_motor.throttle = msg.data.steering * self.steering_gain + self.steering_offset
        self.throttle_motor.throttle = msg.data.throttle * self.throttle_gain + self.throttle_offset


def main(args=None):
    rclpy.init(args=args)

    servo_driver = ServoPca9685Driver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()