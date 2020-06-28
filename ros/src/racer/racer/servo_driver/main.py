from servo_driver.servo_driver import ServoDriver
import rclpy

def main(args=None):
    rclpy.init(args=args)

    servo_driver = ServoDriver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()