import cv2
import numpy as np
import matplotlib.pyplot as plt

from matplotlib_visualizer import MatplotlibVisualizer
from operation import Operation   
from find_lines_with_sliding_windows_op import FindLinesWithSlidingWindowsOp        

class GetGrayscaleImageOp(Operation):
    def _run(self, input):
        self._check_input(input, ['bgr_image'])        
        input['binary_image'] = cv2.cvtColor(input['bgr_image'], cv2.COLOR_BGR2GRAY)

class CropImageOp(Operation):
    def __init__(self, y_min, y_max):
        self.y_min_= y_min
        self.y_max_= y_max

    def _run(self, input):
        self._check_input(input, ['bgr_image'])        
        input['bgr_image'] = input['bgr_image'][self.y_min_:self.y_max_, :, :]

class ApplyHistogramEqualizationOp(Operation):
    def _run(self, input):
        self._check_input(input, ['binary_image'])       
        input['binary_image'] = cv2.equalizeHist(input['binary_image'])

class ApplySobelFilterOp(Operation):
    def __init__(self, kernel_size, magnitude_threshold):
        self.kernel_size_ = kernel_size
        self.magnitude_threshold_ = magnitude_threshold

    def _run(self, input):
        self._check_input(input, ['binary_image'])

        sobel_x = cv2.Sobel(input['binary_image'], cv2.CV_64F, 1, 0, ksize=self.kernel_size_)
        sobel_y = cv2.Sobel(input['binary_image'], cv2.CV_64F, 0, 1, ksize=self.kernel_size_)

        sobel_mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
        sobel_mag = np.uint8(sobel_mag / np.max(sobel_mag) * 255)

        _, result = cv2.threshold(sobel_mag, thresh=self.magnitude_threshold_, maxval=1, type=cv2.THRESH_BINARY)        
        input['binary_image'] = np.logical_or(input['binary_image'], result)

class ApplyBinaryThresholdOp(Operation):
    def __init__(self , threshold):
        self.threshold_ = threshold

    def _run(self, input):
        self._check_input(input, ['binary_image'])                
        _, input['binary_image'] = cv2.threshold(input['binary_image'], thresh=self.threshold_, maxval=255, type=cv2.THRESH_BINARY)

class FillGapsWithMorphologyOp(Operation):
    def __init__(self, kernel_size):
        self.kernel_size_ = kernel_size        

    def _run(self, input):
        self._check_input(input, ['binary_image'])

        kernel = np.ones((self.kernel_size_,  self.kernel_size_), np.uint8)
        input['binary_image'] = cv2.morphologyEx(input['binary_image'].astype(np.uint8), cv2.MORPH_CLOSE, kernel)

class ApplyBirdeyeTransformOp(Operation):
    def __init__(self, top_to_bottom_ratio=0.35):
        self.top_to_bottom_ratio_ = top_to_bottom_ratio  

    def _run(self, input):
        self._check_input(input, ['binary_image'])
                
        h, w = input['binary_image'].shape
        top_offset = (1 - self.top_to_bottom_ratio_ ) * w / 2

        source_polygon = np.float32([[w, h], [0, h], [top_offset, 0], [w - top_offset, 0]])
        target_polygon = np.float32([[w, h], [0, h], [0, 0], [w, 0]])

        translation_matrix = cv2.getPerspectiveTransform(source_polygon, target_polygon)        

        input['birdeye_binary_image'] = cv2.warpPerspective(input['binary_image'], 
            translation_matrix, (w, h), flags=cv2.INTER_LINEAR)

class CalculateCTEOp(Operation):
    def _run(self, input):
        self._check_input(input, ['left_line', 'right_line', 'birdeye_binary_image'])        

        height, width = input['birdeye_binary_image'].shape
        current_center_x = width / 2.0
        left_x = input['left_line'].get_values([height])[0]
        right_x = input['right_line'].get_values([height])[0]
        target_center_x = (left_x + right_x) / 2.0        
        
        input['cte_error'] = target_center_x - current_center_x

class CentralLineExtractionPipeline:

    def __init__(self, ops):
        self.ops_ = ops    

    def run(self, bgr_image, visualizer=None):
        input = {'bgr_image': bgr_image}
        for op in self.ops_:
            op.run(input, visualizer)
        return input['cte_error'], input['birdeye_binary_image'], input['left_line'], input['right_line']

# TODO: Make visualization dependent on pipeline contents, once smth is gone from pipeline I should not viz it
if __name__ == '__main__':
    bgr_image = cv2.imread('2_.png')    
    visualizer = MatplotlibVisualizer(bgr_image)
    pipeline = CentralLineExtractionPipeline([
        CropImageOp(110, 160),
        GetGrayscaleImageOp(),        
        ApplyBinaryThresholdOp(200),
        ApplySobelFilterOp(5, 50),
        FillGapsWithMorphologyOp(5),
        ApplyBirdeyeTransformOp(),
        FindLinesWithSlidingWindowsOp(),
        CalculateCTEOp()])    
    cte,_,_,_ = pipeline.run(bgr_image, visualizer)
    print(f"CTE ={cte}")
    visualizer.show()