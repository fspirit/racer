from servo_driver import ServoDriver
<<<<<<< HEAD
from adafruit_servokit import ServoKit
=======
>>>>>>> a0f14b2... Fix import path
import rclpy

def main(args=None):
    rclpy.init(args=args)

    servo_kit = ServoKit(channels=16, i2c_address=0x40)

    steering_channel_index = 0
    steering_motor = servo_kit.continuous_servo[steering_channel_index]
    throttle_channel_index = 1
    throttle_motor = servo_kit.continuous_servo[throttle_channel_index]

    servo_driver = ServoDriver(steering_motor, throttle_motor)

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()