import rclpy
import numpy as np
import time
import traceback
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty
from std_msgs.msg import String
from remembr.memory.milvus_memory import MilvusMemory
from remembr.agents.remembr_agent import ReMEmbRAgent
from scipy.spatial.transform import Rotation as R

import threading

from common_utils import format_pose_msg


class AgentNode(Node):

    def __init__(self):
        super().__init__("AgentNode")

        # self.declare_parameter("llm_type", "command-r")
        # self.declare_parameter("llm_type", "codestral")
        self.declare_parameter("llm_type", "llama3.1:8b")
        # self.declare_parameter("llm_type", "llama3.3")
        self.declare_parameter("db_collection", "test_collection")
        self.declare_parameter("db_ip", "127.0.0.1")
        # self.declare_parameter("db_ip", "172.17.12.82")
        self.declare_parameter("query_topic", "/speech")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("goal_text_topic", "/goal_response")
        self.declare_parameter("terminate_topic", "/terminate_remembr")
        self.declare_parameter("status_topic", "/remembr_status")
        self.declare_parameter("video_name_topic", "/video_name")

        # look for "robot" keyword
        self.query_filter = lambda text: "robot" in text.lower()

        self.query_subscriber = self.create_subscription(
            String,
            self.get_parameter("query_topic").value,
            self.query_callback,
            10
        )

        self.query_thread = None

        self.terminate_subscriber = self.create_subscription(
            Empty,
            self.get_parameter("terminate_topic").value,
            self.terminate_callback,
            1
        )

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self.pose_callback,
            10
        )

        self.video_subscriber = self.create_subscription(
            String,
            self.get_parameter("video_name_topic").value,
            self.change_db_callback,
            10
        )

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            10
        )
        
        self.goal_text_publisher = self.create_publisher(
            String,
            self.get_parameter("goal_text_topic").value,
            10
        )
        
        self.status_topic_timer = self.create_timer(1, self.status_topic_callback)
        self.status_topic_publisher = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            1
        )

        self.memory = MilvusMemory(
            self.get_parameter("db_collection").value,
            self.get_parameter("db_ip").value
        )
        self.agent = ReMEmbRAgent(
            llm_type=self.get_parameter("llm_type").value
        )
        self.agent.set_memory(self.memory)

        self.last_pose = None
        self.logger = self.get_logger()
        

    def query_callback(self, msg: String):
        
        # if not self.query_filter(msg.data):
        #     logger.info(f"Skipping query {msg.data} because it does not have keyword")
        #     return 

        try:
            query = msg.data

            # Add additional context information to query
            # position = None
            # current_time = None
            # if self.last_pose is not None:
            #     position, angle, current_time = format_pose_msg(self.last_pose)
            #     position = position.tolist()
            #     query +=  f"\nYou are currently located at {position.tolist()} and the time is {current_time}."
            if self.query_thread and self.query_thread.is_alive():
                print("stop previous")
                self.terminate_callback(Empty())
                
            self.query_thread = threading.Thread(target=self._query, args=(query, ))
            self.query_thread.start()

            # # Run the Remembr Agent
            # response = self.agent.query(query)
            
            # # Generate the goal pose from the response
            # position = response.position
            # # quat = R.from_euler('z', response.orientation).as_quat()
            # quat = R.from_euler('z', position[2]).as_quat()
            # position[2] = 0.0
            # quat = np.squeeze(quat)
            # goal_pose = PoseStamped()
            # goal_pose.header.frame_id = 'map'
            # goal_pose.header.stamp = self.get_clock().now().to_msg()
            # goal_pose.pose.position.x = float(position[0])
            # goal_pose.pose.position.y = float(position[1])
            # goal_pose.pose.position.z = float(position[2])
            # goal_pose.pose.orientation.x = float(quat[0])
            # goal_pose.pose.orientation.y = float(quat[1])
            # goal_pose.pose.orientation.z = float(quat[2])
            # goal_pose.pose.orientation.w = float(quat[3])
            # # Publish the result
            # self.goal_pose_publisher.publish(goal_pose)

            # self.logger.info("Query executed: ")
            # self.logger.info("\tresoning: " + response.answer_reasoning)
            # self.logger.info("\tText: " + response.text)
            # self.logger.info(f"\tPosition: {response.position}")
            # self.logger.info(f"\tOrientation: {response.orientation}")
        
        except:
            print("FAILED. Returning")
            print(traceback.format_exc())
            return

    def _query(self, query):
        try:
            position = angle = current_time = None
            if self.last_pose is not None:
                position, angle, current_time = format_pose_msg(self.last_pose)
                position = position.tolist()
            response = self.agent.query(query, position, current_time)
            if response: 
                # Generate the goal pose from the response
                if None in response.position:
                    quat = R.from_euler('z', angle).as_quat()
                    quat = np.squeeze(quat)
                else:
                    position = response.position
                    # quat = R.from_euler('z', response.orientation).as_quat()
                    quat = R.from_euler('z', position[2]).as_quat()
                    quat = np.squeeze(quat)
                    position[2] = 0.0
                if position:
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = float(position[0])
                    goal_pose.pose.position.y = float(position[1])
                    goal_pose.pose.position.z = float(position[2])
                    goal_pose.pose.orientation.x = float(quat[0])
                    goal_pose.pose.orientation.y = float(quat[1])
                    goal_pose.pose.orientation.z = float(quat[2])
                    goal_pose.pose.orientation.w = float(quat[3])
                    # Publish the result
                    self.goal_pose_publisher.publish(goal_pose)
                goal_text = String()
                goal_text.data = response.text
                self.goal_text_publisher.publish(goal_text)

                # self.logger.info("Query executed: ")
                # self.logger.info("\tresoning: " + response.answer_reasoning)
                # self.logger.info("\tText: " + response.text)
                # self.logger.info("\tTarget: " + response.target)
                # self.logger.info(f"\tPosition: {response.position}")
                # self.logger.info(f"\tOrientation: {response.orientation}")
                print("Query executed: ")
                print("\tresoning: " + response.answer_reasoning)
                print("\tText: " + response.text)
                print("\tKeypoint: " + response.keypoint)
                print(f"\tPosition: {response.position}")
                print(f"\tOrientation: {response.orientation}")
        
        except:
            print("FAILED. Returning")
            print(traceback.format_exc())
            return

    def status_topic_callback(self):
        status_msg = String()
        status_msg.data = "idle"
        if self.query_thread and self.query_thread.is_alive():
            status_msg.data = "running"

        self.status_topic_publisher.publish(status_msg)
            
    def terminate_callback(self, _):
        # TODO stop query
        self.agent.terminate()
        self.query_thread.join()
        print("stop query thread")
        
        if self.last_pose:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose = self.last_pose.pose.pose
            self.goal_pose_publisher.publish(goal_pose)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.last_pose = msg

    def change_db_callback(self, msg: String):
        print(f"change_db_callback")
        db_name = f'hm3d_{msg.data.replace("-", "_").replace("hm3d_", "")}'
        print(f"switch to {db_name}")

        self.memory = MilvusMemory(
            db_name,
            self.get_parameter("db_ip").value
        )
        self.agent.set_memory(self.memory)


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # freeze_support()
    main()