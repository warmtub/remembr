import rclpy
import numpy as np
from copy import deepcopy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from scipy.spatial.transform import Rotation as R
from remembr.memory.memory import MemoryItem
from remembr.memory.milvus_memory import MilvusMemory

from common_utils import format_pose_msg



class MemoryBuilderNode(Node):

    def __init__(self):
        super().__init__("MemoryBuilderNode")

        self.declare_parameter("db_collection", "test_collection")
        self.declare_parameter("db_ip", "127.0.0.1")

        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("caption_topic", "/caption")
        self.declare_parameter("caption_pose_topic", "/caption_pose")
        self.declare_parameter("video_name_topic", "/video_name")

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self.pose_callback,
            1
        )

        self.caption_subscriber = self.create_subscription(
            String,
            self.get_parameter("caption_topic").value,
            self.caption_callback,
            10
        )
        
        self.caption_pose_subscriber = self.create_subscription(
            Bool,
            self.get_parameter("caption_pose_topic").value,
            self.caption_pose_callback,
            1
        )
        
        self.video_subscriber = self.create_subscription(
            String,
            self.get_parameter("video_name_topic").value,
            self.change_db_callback,
            10
        )
        
        self.memory = MilvusMemory(
            self.get_parameter("db_collection").value,
            self.get_parameter("db_ip").value
        )
        self.memory.reset()

        self.pose_msg = None
        self.caption_msg = None
        self.caption_pose_msg = None
        self.logger = self.get_logger()

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        
        self.pose_msg = msg

    def caption_callback(self, msg: String):

        if self.caption_pose_msg is not None:

            position, angle, pose_time = format_pose_msg(self.caption_pose_msg)

            memory = MemoryItem(
                caption=msg.data,
                time=pose_time,
                position=position,
                theta=angle
            )

            self.logger.info(f"Added memory item {memory}")

            self.memory.insert(memory)

    def caption_pose_callback(self, msg: Bool):

        if self.pose_msg is not None and msg.data:
            self.caption_pose_msg = deepcopy(self.pose_msg)
            self.caption_pose_msg.header.stamp = self.get_clock().now().to_msg()

    def change_db_callback(self, msg: String):
        db_name = f'hm3d_{msg.data.replace("-", "_").replace("hm3d_", "")}'
        print(f"switch to {db_name}")

        # self.memory = MilvusMemory(
        #     db_name,
        #     self.get_parameter("db_ip").value
        # )
        self.memory = MilvusMemory(
            "test_collection",
            self.get_parameter("db_ip").value
        )
        self.memory.reset()

def main(args=None):
    rclpy.init(args=args)
    node = MemoryBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()