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

class ApplyHistogramEqualizationOp(Operation):
    def _run(self, input):
        self._check_input(input, ['binary_image'])       
        input['binary_image'] = cv2.equalizeHist(input['binary_image'])

class ApplyBinaryThresholdOp(Operation):
    def __init__(self , threshold):
        self.threshold_ = threshold

    def _run(self, input):
        self._check_input(input, ['binary_image'])                
        _, input['binary_image'] = cv2.threshold(input['binary_image'], thresh=self.threshold_, maxval=255, type=cv2.THRESH_BINARY)

class MatplotlibVisualizer():
    def __init__(self):
        self.figure, self.subplots = plt.subplots(1, 3)        

    def visualize(self, op_name, state):
        if op_name == 'GetGrayscaleImageOp':
            self.subplots[0].imshow(state['binary_image'], cmap='gray')
            self.subplots[0].set_title('grayscale')
        if op_name == 'ApplyHistogramEqualizationOp':
            self.subplots[1].imshow(state['binary_image'], cmap='gray')
            self.subplots[1].set_title('equalized_grayscale')
        if op_name == 'ApplyBinaryThresholdOp':
            self.subplots[2].imshow(state['binary_image'], cmap='gray')
            self.subplots[2].set_title('thresholded_grayscale')
    
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

if __name__ == '__main__':
    bgr_image = cv2.imread('2_.png')
    visualizer = MatplotlibVisualizer()
    pipeline = CentralLineExtractionPipeline([GetGrayscaleImageOp(), ApplyHistogramEqualizationOp(), ApplyBinaryThresholdOp(200)])    
    pipeline.run(bgr_image, visualizer)
    visualizer.show()