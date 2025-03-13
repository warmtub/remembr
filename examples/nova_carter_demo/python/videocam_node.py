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
        sequence = 0
        # indir = os.getenv(ENV_CODA_ROOT_DIR)
        # poses_dir = join(indir, "poses", "dense")
        pose_file = Path(f"/ssd/data/{sequence}.txt")
        pose_np = np.fromfile(pose_file, sep=' ').reshape(-1, 8)
        print(pose_np.shape)
        print(len(sorted(Path(f"/ssd/data/2d_rect_{sequence}/cam0/{sequence}/").glob("*.png"))))
        image_files = sorted(Path(f"/ssd/data/2d_rect_{sequence}/cam0/{sequence}/").glob("*.png"), key=lambda x: int(str(x).split("_")[-1].split(".")[0]))
        for image_path, pose in zip(image_files, pose_np):
            print(image_path)
        
            cv2_im = cv2.imread(str(image_path))
            pose_ts, x, y, z, qw, qx, qy, qz = pose
            # base_pose[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
            # base_pose[:3, 3] = [x, y, z]

            # quat = R.from_matrix(rotation).as_quat()
            # quat = np.squeeze(quat)

            # if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv2_im), "bgr8"))
            self.get_logger().info('Publishing an image')

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()  # Use your node's clock
            pose_msg.header.frame_id = "map"  # Frame of reference

            pose_msg.pose.pose.position.x = float(x)
            pose_msg.pose.pose.position.y = float(y)
            pose_msg.pose.pose.position.z = float(z)
            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw
            print(pose_msg.pose.pose)

            string_msg = String()
            string_msg.data = time.strftime("%H:%M:%S", time.gmtime())
            
            
            self.publisher_1.publish(pose_msg)
            # self.publisher_2.publish(string_msg)
            self.i += 1

            time.sleep(0.2)
            
        # time.sleep(10)
            
def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()