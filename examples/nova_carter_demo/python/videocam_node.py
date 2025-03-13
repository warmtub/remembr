#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from pathlib import Path


class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, "/webcam_image", 1)
        self.video_path = 0
        # self.video_cap = cv2.VideoCapture(self.video_path)

        # timer_period = 1/self.video_cap.get(cv2.CAP_PROP_FPS)  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.count = 0
        self.source_folder = "/remembr/data/frames/hm3d-v0/000-hm3d-BFRyYbPCCPE"
       
    # def timer_callback(self):

        for image_path in sorted(Path(self.source_folder).glob("*-rgb.png")):
            print(image_path)
        
            cv2_im = cv2.imread(str(image_path))
            # if self.count == 5:
            #     cv2.imwrite("test.jpg", cv2_im)
            #     self.count = 0
            # else: self.count += 1

            # if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv2_im), "bgr8"))
            self.get_logger().info('Publishing an image')
            time.sleep(1)
            
def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()