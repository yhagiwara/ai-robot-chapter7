from ai_robot_book_interfaces.srv import StringCommand    
    
import rclpy    
from rclpy.node import Node    
    
import random    
from time import sleep
    
class VoiceServer(Node):    
        
    def __init__(self):    
        super().__init__('voice_server')    
        self.srv = self.create_service(    
            StringCommand, 'voice/command', self.command_callback)    
        
    def command_callback(self, request, response):    
        sleep(1)
            
        prob = random.random()    
        self.get_logger().info(f"音声認識をします")    
    
        if 0.5 > prob:    
            self.get_logger().info("音声認識が成功しました")    
            response.answer = "succeeded"
        else:    
            self.get_logger().info("音声認識が失敗しました")    
            response.answer = "failed"
    
        return response    
    
    
def main(args=None):    
    rclpy.init(args=args)    
        
    voice_server = VoiceServer()
        
    rclpy.spin(voice_server)    
    
    rclpy.shutdown()    
    
    
if __name__ == '__main__':    
    main()
