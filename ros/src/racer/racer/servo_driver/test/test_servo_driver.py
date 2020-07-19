import rclpy
import sys, os
sys.path.append(os.path.join(sys.path[0], '..'))

import unittest
from unittest.mock import Mock

from servo_driver import ServoDriver

class TestServoDriver(unittest.TestCase):

    def test_when_node_receives_actuators_then_it_sets_correct_values_to_servo_motors(self):
        steering_motor = Mock()
        throttle_motor = Mock()

        servo_driver = ServoDriver(steering_motor, throttle_motor)

        test_steering_value = 0.5
        test_throttle_value = 0.5

        actuators = Mock()
        actuators.data.steering = test_steering_value
        actuators.data.throttle = test_throttle_value

        servo_driver.handle_command(actuators)

        self.assertEqual(steering_motor.throttle, servo_driver.steering_gain * test_steering_value +  servo_driver.steering_offset)
        self.assertEqual(throttle_motor.throttle, servo_driver.throttle_gain * test_throttle_value +  servo_driver.throttle_offset)

if __name__ == '__main__':
    rclpy.init()

    unittest.main()

    rclpy.shutdown()