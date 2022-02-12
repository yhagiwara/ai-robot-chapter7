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
        self.get_logger().info(request.command)

        sleep(1)
            
        prob = random.random()    
        self.get_logger().info(f"Try to move the target position")    
    
        if 0.7 > prob:    
            self.get_logger().info("Succeeded to move the target position")    
            response.answer = "succeeded"
        else:    
            self.get_logger().info("Failed to move the target position")    
            response.answer = "failed"
    
        return response    
    
    
def main(args=None):    
    rclpy.init(args=args)    
        
    navigation_server = NavigationServer()    
        
    rclpy.spin(navigation_server)
    
    rclpy.shutdown()    
    
    
if __name__ == '__main__':
    main()
