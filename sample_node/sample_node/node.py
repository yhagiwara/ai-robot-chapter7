#!/usr/bin/env python3    
    
import rclpy    
from rclpy.node import Node    
from std_msgs.msg import String    
                                   
                          
class SampleNode:          
    def __init__(self):    
        # Create node                      
        self.node = Node("sample_node")    
                                                
        # Prepare for subscription and create publisher and logger                                
        self.sample_node_subscription = None                                                      
        self.sample_node_publisher = self.node.create_publisher(String, "/sample/response", 1)    
        self.logger = self.node.get_logger()    
                                                                                                  
        self.count = 0                          
                             
    def main(self):                                                                                                  
        # Node loop                                                                                                                  
        while rclpy.ok():                                                                                                            
            # Regist subscription if connection from subscriber.                                                                     
            if self.sample_node_publisher.get_subscription_count() > 0 and self.sample_node_subscription is None:                    
                self.sample_node_subscription = self.node.create_subscription(String, "/sample/request", self.sample_callback, 1)    
                self.logger.info("Regist subscriber")                                                                         
                                                                                                                              
            # Unregist subscription if disconection from subscriber.                                                          
            elif self.sample_node_publisher.get_subscription_count() == 0 and (self.sample_node_subscription is not None):
                self.node.destroy_subscription(self.sample_node_subscription)
                self.sample_node_subscription = None             
                self.logger.info("Destroy subscriber")           
                                                                 
            # Try to spin node at once                           
            try:                                                 
                rclpy.spin_once(self.node, timeout_sec=0.001)    
            except Exception as e:                         
                # destroy the subscription if get exception    
                self.sample_node_subscription.destroy()    
                self.sample_node_subscription = None    
                                                   
    def sample_callback(self, msg):                
        print(msg)                                 
                                                   
        msg = String()                             
        msg.data = f"World {self.count}"           
        self.sample_node_publisher.publish(msg)    
                           
        self.count += 1    
                              
                              
def main():                   
    rclpy.init()              
                              
    SampleNode().main()       
                              
if __name__ == "__main__":    
    main()
