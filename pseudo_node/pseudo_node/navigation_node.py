from ai_robot_book_interfaces.srv import StringCommand    
    
import rclpy    
from rclpy.node import Node    
    
import random    
from time import sleep
    
class NavigationServer(Node):    
        
    def __init__(self):    
        super().__init__('navigation_server')    
        self.srv = self.create_service(    
            StringCommand, 'navigation/command', self.command_callback)    
        
    def command_callback(self, request, response):
        sleep(1)
            
        prob = random.random()    
        self.get_logger().info(f"目標座標へ移動します {request.command}")
    
        if 0.7 > prob:    
            self.get_logger().info("目標座標への移動が成功しました")    
            response.answer = "reached"
        else:    
            self.get_logger().info("目標座標への移動が失敗しました")    
            response.answer = "failed"
    
        return response    
    
    
def main(args=None):    
    rclpy.init(args=args)    
        
    navigation_server = NavigationServer()    
        
    rclpy.spin(navigation_server)
    
    rclpy.shutdown()    
    
    
if __name__ == '__main__':
    main()
