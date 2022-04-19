from airobot_interfaces.srv import StringCommand    
    
import rclpy    
from rclpy.node import Node    
    
import random    
from time import sleep
    
class VisionServer(Node):    
        
    def __init__(self):    
        super().__init__('vision_server')    
        self.srv = self.create_service(    
            StringCommand, 'vision/command', self.command_callback)    
        
    def command_callback(self, request, response):    
            
        prob = random.random()    
        self.get_logger().info(f"目標物体を探します")    

        sleep(1)
    
        if 0.5 > prob:
            self.get_logger().info("目標物体の検出が成功しました")
            response.answer = "detected"
        else:    
            self.get_logger().info("目標物体の検出が失敗しました")    
            response.answer = "failed"
    
        return response    
    
    
def main(args=None):    
    rclpy.init(args=args)    
        
    vision_server = VisionServer()    
        
    rclpy.spin(vision_server)    
    
    rclpy.shutdown()    
