from racer.msg import ControlErrors

from rclpy.node import Node
—class ControlErrorExtractor(Node):

    DEFAULT_FPS = 30

    def __init__(self, factory, pipeline, settings = {}):
        super().__init__('control_error_extractor')
        
        self.fps_ = settings['camera_fps'] if ('camera_fps' in settings) else self.DEFAULT_FPS
        self.camera_ = factory.create_camera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=self.fps_)
        self.timer_ = factory.create_timer(self.extract_control_errors, (1.0 / self.fps_ * 1e9))
        self.pipeline_ = pipeline        
        self.publisher_ = self.create_publisher(String, 'ControlErrors', 10)

    def extract_control_errors(self):
        image = self.camera_.read()
        cte = self.pipeline_.run(image)
        
        msg = ControlErrors()
        msg.data.cross_track_error = cte
        self.publisher_.publish(msg)        


