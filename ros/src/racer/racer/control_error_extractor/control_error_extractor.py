import rclpy
from racer_msgs.msg import ControlErrors
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import utils

class ControlErrorExtractor(Node):

    DEFAULT_FPS = 30

    def __init__(self, factory, pipeline, settings = {}):
        super().__init__('control_error_extractor')
        
        self.fps_ = settings['camera_fps'] if ('camera_fps' in settings) else self.DEFAULT_FPS
        self.camera_ = factory.create_camera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=self.fps_)
        self.timer_ = self.create_timer((1.0 / self.fps_), self.step)
        self.pipeline_ = pipeline        

        self.cte_publisher_ = self.create_publisher(ControlErrors, 'control_errors', 10)
        self.original_img_publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.final_birdeye_img_publisher_ = self.create_publisher(Image, 'final_birdeye_image', 10)

    def publish_original_image(self, bgr_image):
        msg = self.ros_msg_from_bgr_image(bgr_image)
        self.original_img_publisher_.publish(msg)

    def publish_birdeye_image_with_lines(self, birdeye_binary_image, left_line, right_line):
        birdeye_bgr_image = utils.create_birdeye_image_with_lines(birdeye_binary_image, left_line, right_line)
        msg = self.ros_msg_from_bgr_image(birdeye_bgr_image)
        self.final_birdeye_img_publisher_.publish(msg)
        
    def publish_cte(self, cte):
        msg = ControlErrors()        
        msg.cross_track_error = cte
        self.cte_publisher_.publish(msg)

    def ros_msg_from_bgr_image(self, bgr_image):
        print(bgr_image.dtype)
        msg = Image(encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/map"

        msg.height, msg.width, _ = bgr_image.shape

        continious_array = np.ascontiguousarray(bgr_image)

        msg.data = continious_array.tostring()
        msg.step = continious_array.strides[0]
        msg.is_bigendian = (
            bgr_image.dtype.byteorder == '>' or 
            bgr_image.dtype.byteorder == '=' and sys.byteorder == 'big'
        )        

        return msg

    def step(self):
        bgr_image = self.camera_.read()

        self.publish_original_image(bgr_image)

        cte, birdeye_binary_image, left_line, right_line = self.pipeline_.run(bgr_image)        

        self.publish_birdeye_image_with_lines(birdeye_binary_image, left_line, right_line)
        self.publish_cte(cte)
