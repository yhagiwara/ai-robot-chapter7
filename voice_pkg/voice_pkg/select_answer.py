import rclpy
import rclpy.node
from std_msgs.msg import String


class SelectAnswer(rclpy.node.Node):    
    def __init__(self):    
        super().__init__("select_answer")
    
        self.logger = self.get_logger()    
        self.logger.info("Start selection answer")

        self.text_sub = self.create_subscription(String, "recognized_text", self.text_subscribe, 10)
        self.answer_pub = self.create_publisher(String, "answer_text", 10)
    
    def text_subscribe(self, msg):
        self.logger.info("Subscribe text '{}'".format(msg.data))

        self.select_answer()

    def select_answer(self):
        self.logger.info("Select answer")
        msg = String()
        msg.data = "I will do"

        self.answer_pub.publish(msg)
        self.logger.info("Published answer text '{}'".format(msg.data))
    
    
def main():    
    rclpy.init()    
    
    select_answer = SelectAnswer()

    try:
        rclpy.spin(select_answer)
    except:
        select_answer.destroy_node()

    rclpy.shutdown()
