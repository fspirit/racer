import numpy as np
import cv2

def generate_sample_points_for_lines(image_height, left_line, right_line):        
    sample_y = np.linspace(0, image_height - 1, image_height)
    left_line_sample_x = left_line.get_values(sample_y)
    right_line_sample_x = right_line.get_values(sample_y)

    return sample_y, left_line_sample_x, right_line_sample_x

def draw_polyline(bgr_image, x, y, color):
    points = np.array(list(zip(x, y)))
    cv2.polylines(bgr_image, [points], False, color)

def create_birdeye_image_with_lines(birdeye_binary_image, left_line, right_line):
    sample_y, left_line_sample_x, right_line_sample_x = generate_sample_points_for_lines(birdeye_binary_image.shape[0], left_line, right_line) 
    center_line_sample_x = [(a + b) / 2.0 for a, b in zip(left_line_sample_x, right_line_sample_x)]

    red_bgr_color = (0, 0, 255)
    yellow_bgr_color = (20, 100, 100)

    birdeye_rgb_image = np.dstack((birdeye_binary_image, birdeye_binary_image, birdeye_binary_image)) * 255 

    draw_polyline(birdeye_rgb_image, left_line_sample_x, sample_y, red_bgr_color)
    draw_polyline(birdeye_rgb_image, left_line_sample_x, sample_y, red_bgr_color)
    draw_polyline(birdeye_rgb_image, center_line_sample_x, sample_y, yellow_bgr_color)

    return birdeye_rgb_image

if __name__ == '__main__':
    import os

    bgr_image = cv2.imread(os.path.dirname(__file__) + "/2_.png")     
    line_x = [0, 200]
    line_y = [0, 100]
    draw_polyline(bgr_image, line_x, line_y, (255, 0, 0))
    cv2.imwrite(os.path.dirname(__file__)+ "/2_changed.png", bgr_image) 

