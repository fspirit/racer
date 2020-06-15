import rclpy
from rclpy.timer import Timer

from jetcam.csi_camera import CSICamera

from il_dataset_recorder.il_dataset_recorder import ILDatasetRecorder

class Factory():

    def create_timer(self, callback, timer_period_ns):
        return Timer(callback = callback, timer_period_ns=timer_period_ns)

    def create_camera(self, width, height, capture_width, capture_height, capture_fps):
        return CSICamera(width=width, height=height, capture_width=capture_width, capture_height=capture_height, capture_fps=capture_fps)


def main(args=None):
    rclpy.init(args=args)

    factory = Factory()
    settings = {'camera_fps': 2}

    il_dataset_recorder = ILDatasetRecorder(factory, settings)

    rclpy.spin(il_dataset_recorder)

    il_dataset_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()