#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np
import time
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import json
import threading


class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')
        self.video_name = ""
        self.publisher_1 = self.create_publisher(String, 'speech', 1)
        self.publisher_3 = self.create_publisher(String, 'video_name', 1)
        self.subscriber_1 = self.create_subscription(String, 'goal_text', self.goal_callback, 1)
        self.ready = False
        t = threading.Thread(target=self.ask_and_wait)
        t.start()

    def ask_and_wait(self):

        with open("open-eqa-localization.json", "r") as jf:
            questions = json.load(jf)

        for ques in questions[50:]:
            self.ready = False
            print(ques)
            string_msg = String()
            string_msg.data = ques["episode_history"].split("/")[-1]
            self.publisher_3.publish(string_msg)
            time.sleep(10)
            string_msg2 = String()
            string_msg2.data = ques["question"]
            self.publisher_1.publish(string_msg2)

            while not self.ready:
                time.sleep(1)

            break

        rclpy.shutdown()

    def goal_callback(self, msg: String):
        print(msg.data)
        self.ready = True
    


def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()