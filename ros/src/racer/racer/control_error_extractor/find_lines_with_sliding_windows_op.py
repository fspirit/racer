import cv2
import numpy as np

from operation import Operation

class Polyline:    
    def __init__(self, x, y, order):
        self.order = order
        self.coeffs = list(reversed(np.polyfit(x, y, order)))
        
    def get_values(self, x):
        return [sum([(dx**i)*c for i,c in enumerate(self.coeffs)]) for dx in x]
            
    def create_from_points(x, y, order=2):
        if not list(x) or not list(y):
            return None
        return Polyline(x, y, order)

class Window:        
    def __init__(self, width, height, bottom_midpoint_x, bottom_y):
        self.bottom_y = bottom_y
        self.top_y = bottom_y - height
        self.right_x = bottom_midpoint_x + width // 2
        self.left_x = bottom_midpoint_x - width // 2
        
    
    def get_nonzero_pixels_indices(self, nonzero_x, nonzero_y):
        mask = ((nonzero_y >= self.top_y) & 
                (nonzero_y < self.bottom_y) & 
                (nonzero_x >= self.left_x)& 
                (nonzero_x < self.right_x))
        
        return nonzero_x[mask], nonzero_y[mask]                 

class FindLinesWithSlidingWindowsOp(Operation):
    def __init__(self, windows_count = 5, window_width = 200, min_pix_to_recenter = 30):
        self.windows_count_ = windows_count
        self.window_width_ = window_width
        self.min_pix_to_recenter_ = min_pix_to_recenter

    def find_base_for_lines(self, birdeye_binary):        
        height, _ = birdeye_binary.shape
        
        histogram = np.sum(birdeye_binary[height//4:-30, :], axis=0)
        
        midpoint = len(histogram) // 2    
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        return leftx_base, rightx_base

    def find_line_with_sliding_windows(self, nonzero_x, nonzero_y, starting_x, height):                        
        current_x = starting_x
        
        window_height = height // self.windows_count_ 
        
        lane_x = []
        lane_y = []

        for i in range(self.windows_count_):
            window = Window(self.window_width_, window_height, current_x, height - i * window_height)

            x, y = window.get_nonzero_pixels_indices(nonzero_x, nonzero_y)

            lane_x.append(x)
            lane_y.append(y)
            
            if len(x) > self.min_pix_to_recenter_:
                current_x = np.int(np.mean(x))
            
        return Polyline.create_from_points(np.concatenate(lane_y), np.concatenate(lane_x))

    def _run(self, input):
        self._check_input(input, ['birdeye_binary_image'])   

        height, _ = input['birdeye_binary_image'].shape
        
        leftx_base, rightx_base = self.find_base_for_lines(input['birdeye_binary_image'])

        nonzero_y, nonzero_x  = input['birdeye_binary_image'].nonzero()
        
        input['left_line'] = self.find_line_with_sliding_windows(nonzero_x, nonzero_y, leftx_base, height)
        input['right_line'] = self.find_line_with_sliding_windows(nonzero_x, nonzero_y, rightx_base, height)