import numpy as np
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String

class AsrNode(Node):
    def __init__(self):
        super().__init__('AsrNode')
        self.declare_parameter("model", "small.en")
        self.declare_parameter("backend", "whisper_trt")
        self.declare_parameter("audio_topic", "/audio_path")
        self.declare_parameter("speech_topic", "/speech")

        self.query_subscriber = self.create_subscription(
            ByteMultiArray,
            self.get_parameter("audio_topic").value,
            self.query_callback,
            10
        )

        self.speech_publisher = self.create_publisher(
            String, 
            self.get_parameter("speech_topic").value, 
            10
        )

        self.logger = self.get_logger()
        
    def start_asr(self):
        self.model_name = self.get_parameter("model").value
        self.backend = self.get_parameter("backend").value
        self.model_path = None

        if self.backend == "whisper_trt":
            from whisper_trt import load_trt_model
            self.model = load_trt_model(self.model_name, path=self.model_path)
        elif self.backend == "whisper":
            from whisper import load_model
            self.model = load_model(self.model_name)
        elif self.backend == "faster_whisper":
            from faster_whisper import WhisperModel
            class FasterWhisperWrapper:
                def __init__(self, model):
                    self.model_name = model
                def transcribe(self, audio):
                    segs, info = self.model_name.transcribe(audio)
                    text = "".join([seg.text for seg in segs])
                    return {"text": text}
            self.model = FasterWhisperWrapper(WhisperModel(self.model_name))
        
        # warmup
        self.model.transcribe(np.zeros(1536, dtype=np.float32))
        self.logger.info("asr node ready")

    def query_callback(self, msg):
        file_wav = 'temp.wav'
        audio_bytes = b''.join(msg.data)
        with open(file_wav, 'wb') as audio_file:
            audio_file.write(audio_bytes)
        
        # file_wav = msg.data
        text = self.model.transcribe(file_wav)['text']
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)
        self.logger.info("published " + text)
        os.remove(file_wav)

def main(args=None):
    rclpy.init(args=args)
    node = AsrNode()

    node.start_asr()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
