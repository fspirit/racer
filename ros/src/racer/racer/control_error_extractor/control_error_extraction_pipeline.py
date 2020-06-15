import cv2
import numpy as np
import matplotlib.pyplot as plt


class MissingOperationInput(Exception):
    def __init__(self, missing_parameter_key, op_name):
        self.missing_parameter_key_ = missing_parameter_key
        self.op_name_ = op_name

class Operation:
    def _check_input(self, input, keys):        
        for key in keys:
            if key not in input:
                raise MissingOperationInput(key, self.__class__.__name__)        

    def _run(self, input):
        pass

    def run(self, input, visualizer=None):
        self._run(input)
        if (visualizer):
            visualizer.visualize(self.__class__.__name__, input)            

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

class MatplotlibVisualizer():
    def __init__(self, original_bgr_image):
        self.figure, self.subplots = plt.subplots(2, 3)  

        self.subplots[0, 0].imshow(original_bgr_image)
        self.subplots[0, 0].set_title('Original')      

    # TODO: Add param values into Plot Title
    def visualize(self, op_name, state):
        if op_name == 'GetGrayscaleImageOp':
            self.subplots[0, 1].imshow(state['binary_image'], cmap='gray')
            self.subplots[0, 1].set_title('Grayscale')
        if op_name == 'ApplyHistogramEqualizationOp':
            self.subplots[0, 2].imshow(state['binary_image'], cmap='gray')
            self.subplots[0, 2].set_title('Grayscale After Using Histogram Equalization')
        if op_name == 'ApplyBinaryThresholdOp':
            self.subplots[1, 0].imshow(state['binary_image'], cmap='gray')
            self.subplots[1, 0].set_title('Grayscale After Threshold Appication')
        if op_name == 'ApplySobelFilterOp':
            self.subplots[1, 1].imshow(state['binary_image'], cmap='gray')
            self.subplots[1, 1].set_title('Grayscale After Sobel Filter')
        if op_name == 'FillGapsWithMorphologyOp':
            self.subplots[1, 2].imshow(state['binary_image'], cmap='gray')
            self.subplots[1, 2].set_title('Grayscale After Filling Gaps')        
    
    def show(self):
        plt.show()

class CentralLineExtractionPipeline:

    def __init__(self, ops):
        self.ops_ = ops    

    def run(self, bgr_image, visualizer=None):
        input = {'bgr_image': bgr_image}
        for op in self.ops_:
            op.run(input, visualizer)
        # return input['cte_error']

# TODO: Make visualization dependent on pipeline contents, once smth is gone from pipeline I should not viz it
if __name__ == '__main__':
    bgr_image = cv2.imread('2_.png')
    visualizer = MatplotlibVisualizer(bgr_image)
    pipeline = CentralLineExtractionPipeline([
        CropImageOp(110, 160),
        GetGrayscaleImageOp(), 
        # ApplyHistogramEqualizationOp(), 
        ApplyBinaryThresholdOp(200),
        ApplySobelFilterOp(5, 50),
        FillGapsWithMorphologyOp(5)])    
    pipeline.run(bgr_image, visualizer)
    visualizer.show()