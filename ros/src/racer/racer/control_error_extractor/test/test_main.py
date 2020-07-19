import rclpy
import cv2
import os, sys

sys.path.append(os.path.join(sys.path[0], '..'))

from control_error_extractor import ControlErrorExtractor
from control_error_extraction_pipeline import *

from unittest.mock import Mock

class TestFactory():

    def create_camera(self, width, height, capture_width, capture_height, capture_fps):
        camera = Mock()
        
        bgr_image = cv2.imread(os.path.join(sys.path[0], 'sample.png'))

        camera.read.return_value = bgr_image

        return camera

def main(args=None):
    rclpy.init(args=args)

    factory = TestFactory()
    settings = {'camera_fps': 2}    
    
    pipeline = CentralLineExtractionPipeline([
        CropImageOp(110, 160),
        GetGrayscaleImageOp(),        
        ApplyBinaryThresholdOp(200),
        ApplySobelFilterOp(5, 50),
        FillGapsWithMorphologyOp(5),
        ApplyBirdeyeTransformOp(),
        FindLinesWithSlidingWindowsOp(),
        CalculateCTEOp()])  

    cte_extractor = ControlErrorExtractor(factory, pipeline, settings)

    rclpy.spin(cte_extractor)

    cte_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
