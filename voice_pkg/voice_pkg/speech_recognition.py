import rclpy
import rclpy.node
from std_msgs.msg import String

from time import sleep

class SpeechRecognition(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_recognition")

        self.logger = self.get_logger()
        self.logger.info("Start speech recognition")

        self.recognized_pub = self.create_publisher(String, "recognized_text", 10)

        # [TODO]
        #
        # 音声認識を行うプログラムを作成

        sleep(5)
        self.test("Bring me a bottle from dining")

    def test(self, text):
        msg = String()
        msg.data = text

        self.recognized_pub.publish(msg)
        self.logger.info("Published recognized text '{}'".format(text))


def main():
    rclpy.init()

    speech_recognition = SpeechRecognition()

    try:
        rclpy.spin(speech_recognition)
    except:
        speech_recognition.destroy_node()

    rclpy.shutdown()
