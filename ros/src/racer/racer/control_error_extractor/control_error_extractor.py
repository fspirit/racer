from racer_msgs.msg import ControlErrors
from rclpy.node import Node

from sensor_msgs.msg import Image
import numpy as np
import utils

class ControlErrorExtractor(Node):

    DEFAULT_FPS = 30

    def __init__(self, factory, pipeline, cv_bridge, settings = {}):
        super().__init__('control_error_extractor')
        
        self.fps_ = settings['camera_fps'] if ('camera_fps' in settings) else self.DEFAULT_FPS
        self.camera_ = factory.create_camera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=self.fps_)
        self.timer_ = factory.create_timer(self.step, (1.0 / self.fps_ * 1e9))
        self.pipeline_ = pipeline
        self.cv_bridge = cv_bridge

        self.cte_publisher_ = self.create_publisher(ControlErrors, 'control_errors', 10)
        self.original_img_publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.final_birdeye_img_publisher_ = self.create_publisher(Image, 'final_birdeye_image', 10)

    def publish_original_image(self, bgr_image):
        self.original_img_publisher_.publish(self.cv_bridge.cv2_to_imgmsg(bgr_image, "bgr8"))

    def publish_birdeye_image_with_lines(self, birdeye_binary_image, left_line, right_line):
        birdeye_bgr_image = utils.create_birdeye_image_with_lines(birdeye_binary_image, left_line, right_line)
        self.final_birdeye_img_publisher_.publish(self.cv_bridge.cv2_to_imgmsg(birdeye_bgr_image, "bgr8"))
        
    def publish_cte(self, cte):
        msg = ControlErrors()
        msg.data.cross_track_error = cte
        self.cte_publisher_.publish(msg)

    def step(self):
        bgr_image = self.camera_.read()

        self.publish_original_image(bgr_image)

        cte, birdeye_binary_image, left_line, right_line = self.pipeline_.run(image)

        self.publish_birdeye_image_with_lines(birdeye_binary_image, left_line, right_line)
        self.publish_cte(cte)