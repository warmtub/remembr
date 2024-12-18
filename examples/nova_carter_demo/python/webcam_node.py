#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, "/webcam_image", 1)
        self.video_path = 0
        self.video_cap = cv2.VideoCapture(self.video_path)

        timer_period = 1/self.video_cap.get(cv2.CAP_PROP_FPS)  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
       
    def timer_callback(self):

        ret, cv2_im = self.video_cap.read()
        if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv2_im), "bgr8"))
            self.get_logger().info('Publishing an image')

    def video_loop(self):
       
        while(self.video_cap.isOpened()):
            ret, cv2_im = self.video_cap.read()
            if not ret: break
            self.cv_image = cv2_im

def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()