from control_error_extraction_pipeline import CentralLineExtractionPipeline

from rclpy.node import Node
  
class ControlErrorExtractor(Node):

    FPS = 30

    def __init__(self, factory, settings = {}):
        super().__init__('il_dataset_recorder')

        self.camera_ = factory.create_camera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=self.fps_)
        self.timer_ = factory.create_timer(self.extract_control_errors, (1.0 / self.FPS * 1e9))

    def extract_control_errors(self):
        image = self.camera_.read()


        # https://github.com/SlothFriend/CarND-Term1-P4
        # 1 crop
        # 2 filter by color/intencity
        # 3 move into eagle view
        # 4 sliding window centerline detectio
        # 5 curve fitting
        # 6 cte ans heading error calc

        ## Open question: 
        # send 


