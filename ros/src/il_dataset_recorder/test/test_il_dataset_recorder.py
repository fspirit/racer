import rclpy
import sys, os
sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../il_dataset_recorder'))

import unittest
from unittest.mock import Mock

from il_dataset_recorder import ILDatasetRecorder

class TestILDatasetRecorder(unittest.TestCase):

    def test_when_not_recoring_then_no_camera_or_timer_are_create(self):
        factory = Mock()
        settings = {}

        recorder_node = ILDatasetRecorder(factory, settings)

        actuators = Mock()
        recorder_node.handle_actuators(actuators)

        factory.create_timer.assert_not_called()
        factory.create_camera.assert_not_called()


if __name__ == '__main__':
    rclpy.init()

    unittest.main()

    rclpy.shutdown()


