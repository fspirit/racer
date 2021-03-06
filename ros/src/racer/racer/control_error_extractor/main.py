import rclpy

from jetcam.csi_camera import CSICamera

from control_error_extractor import ControlErrorExtractor
from control_error_extraction_pipeline import CentralLineExtractionPipeline

class Factory():

    def create_camera(self, width, height, capture_width, capture_height, capture_fps):
        return CSICamera(width=width, height=height, capture_width=capture_width, capture_height=capture_height, capture_fps=capture_fps)


def main(args=None):
    rclpy.init(args=args)

    factory = Factory()
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