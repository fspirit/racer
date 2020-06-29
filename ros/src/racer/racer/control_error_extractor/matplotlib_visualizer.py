import matplotlib.pyplot as plt
import numpy as np

class MatplotlibVisualizer():
    def __init__(self, original_bgr_image):
        self.figure, self.subplots = plt.subplots(2, 5)  

        self.subplots[0, 0].imshow(original_bgr_image)
        self.subplots[0, 0].set_title('Original')
        

    def _plot_lines(self, state, subplot):
        birdeye_binary = state['birdeye_binary_image']
        height, _ = birdeye_binary.shape
        
        birdeye_rgb_image = np.dstack((birdeye_binary, birdeye_binary, birdeye_binary)) * 255  
            
        ploty = np.linspace(0, height - 1, height)
        left_fitx = state['left_line'].get_values(ploty)
        right_fitx = state['right_line'].get_values(ploty) 
        target_x = [(a + b) / 2.0 for a, b in zip(left_fitx, right_fitx)]

        subplot.imshow(birdeye_rgb_image)                                                          
        subplot.plot(left_fitx, ploty, color='yellow')
        subplot.plot(target_x, ploty, color='yellow') 
        subplot.set_title('Birdeye With Detected Lines')
        subplot.plot(right_fitx, ploty, color='yellow') 


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
        if op_name == 'TransformToBirdeyeOp':
            self.subplots[1, 3].imshow(state['birdeye_binary_image'], cmap='gray')
            self.subplots[1, 3].set_title('Birdeye')   
        if op_name == 'FindLinesWithSlidingWindowsOp':
            self._plot_lines(state, self.subplots[1, 4])

    def show(self):
        plt.show()