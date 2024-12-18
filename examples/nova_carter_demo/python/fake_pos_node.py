#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 1)
        self.publisher_2 = self.create_publisher(String, 'time_string', 1)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.im_list = []
        
    def timer_callback(self):


        # Create an instance of PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()

        # Fill the header
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Use your node's clock
        pose_msg.header.frame_id = "map"  # Frame of reference

        # Fill the pose (position and orientation)
        pose_msg.pose.pose.position.x = float(self.i)
        pose_msg.pose.pose.position.y = float(self.i)
        pose_msg.pose.pose.position.z = float(self.i)

        pose_msg.pose.pose.orientation.x = 0.707
        pose_msg.pose.pose.orientation.y = 0.707
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 0.0  # Quaternion (identity rotation)

        string_msg = String()
        string_msg.data = time.strftime("%H:%M:%S", time.gmtime())
        
        
        self.publisher_.publish(pose_msg)
        self.publisher_2.publish(string_msg)
        self.i += 1
        

    def video_loop(self):
       
        cap = cv2.VideoCapture(self.video_path)
        while(cap.isOpened()):
            ret, cv2_im = cap.read()
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