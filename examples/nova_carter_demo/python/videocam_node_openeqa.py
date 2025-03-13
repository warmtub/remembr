#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np
import time
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# video_list = ['001-hm3d-TPhiubUHKcP', '002-hm3d-wcojb4TFT35', '003-hm3d-c5eTyR3Rxyh', '004-hm3d-66seV3BWPoX', '005-hm3d-yZME6UR9dUN', '006-hm3d-q3hn1WQ12rz', '007-hm3d-bxsVRursffK', '008-hm3d-SiKqEZx7Ejt', '010-hm3d-5cdEh9F2hJL', '011-hm3d-bzCsHPLDztK', '012-hm3d-XB4GS9ShBRE', '013-hm3d-svBbv1Pavdk', '014-hm3d-rsggHU7g7dh', '015-hm3d-5jp3fCRSRjc', '016-hm3d-nrA1tAA17Yp', '017-hm3d-Dd4bFSTQ8gi', '018-hm3d-dHwjuKfkRUR', '019-hm3d-y9hTuugGdiq', '030-hm3d-RJaJt8UjXav', '031-hm3d-Nfvxx8J5NCo', '032-hm3d-6s7QHgap2fW',
# '033-hm3d-vd3HHTEpmyA', '035-hm3d-BAbdmeyTvMZ', '036-hm3d-rJhMRvNn4DS', '037-hm3d-FnSn2KSrALj', '046-hm3d-X4qjx5vquwH', '048-hm3d-kJJyRFXVpx2', '049-hm3d-SUHsP6z2gcJ', '050-hm3d-cvZr5TUy5C5', '072-hm3d-a8BtkwhxdRV', '073-hm3d-LEFTm3JecaC']
video_list = ['001-hm3d-TPhiubUHKcP']

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
        self.source_folder = "/remembr/001-hm3d-TPhiubUHKcP"
        
        self.publisher_1 = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 1)
        # self.publisher_2 = self.create_publisher(String, 'time_string', 1)
        self.publisher_3 = self.create_publisher(String, 'video_name', 1)
        self.i = 0

        # def timer_callback(self):
        for source_folder in video_list:

            # source_folder = "frames/hm3d-v0"+source_folder
            # time.sleep(5)
            # break
            string_msg = String()
            string_msg.data = source_folder
            self.publisher_3.publish(string_msg)
            
            # for image_path in sorted(Path("frames", "hm3d-v0", source_folder).glob("*-rgb.png")):
            for image_path in sorted(Path("/ssd/data/2d_rect/cam0/0/").glob("*.png")):
                print(image_path)
            
                cv2_im = cv2.imread(str(image_path))
                txt_path = str(image_path).replace("-rgb.png", ".txt")
                rotation = []
                position = []
                print(txt_path)
                pose_ts, x, y, z, qw, qx, qy, qz = pose
                base_pose[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
                base_pose[:3, 3] = [x, y, z]

                with open(txt_path, "r") as f:
                    for _ in range(3):
                        data = f.readline()
                        print(data)
                        data = [float(d) for d in data.rstrip().split(" ")]
                        rotation.append(data[:3])
                        position.append(data[3])
                
                rotation = np.array(rotation)
                position = np.array(position)
                rotation = np.roll(rotation, 1, 0)
                rotation = np.roll(rotation, 1, 1)
                position = np.roll(position, 1)
                print(position)

                quat = R.from_matrix(rotation).as_quat()
                quat = np.squeeze(quat)

                # if ret:
                self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv2_im), "bgr8"))
                self.get_logger().info('Publishing an image')

                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()  # Use your node's clock
                pose_msg.header.frame_id = "map"  # Frame of reference

                pose_msg.pose.pose.position.x = float(position[0])
                pose_msg.pose.pose.position.y = float(position[1])
                pose_msg.pose.pose.position.z = float(position[2])
                pose_msg.pose.pose.orientation.x = quat[0]
                pose_msg.pose.pose.orientation.y = quat[1]
                pose_msg.pose.pose.orientation.z = quat[2]
                pose_msg.pose.pose.orientation.w = quat[3]

                string_msg = String()
                string_msg.data = time.strftime("%H:%M:%S", time.gmtime())
                
                
                self.publisher_1.publish(pose_msg)
                # self.publisher_2.publish(string_msg)
                self.i += 1

                time.sleep(2)
                
            time.sleep(10)
            
def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()