import rclpy
from rclpy.node import Node
import smach

from airobot_interfaces.srv import StringCommand


class Bringme_state(Node):
    def __init__(self):
        super().__init__("bringme_state")

    def execute(self):
        sm = smach.StateMachine(outcomes=["succeeded"])

        with sm:
            smach.StateMachine.add("VOICE", Voice(self), transitions={"succeeded": "NAVIGATION", "failed": "VOICE"})
            smach.StateMachine.add("NAVIGATION", Navigation(self), transitions={"succeeded": "VISION", "failed": "NAVIGATION"})
            smach.StateMachine.add("VISION", Vision(self), transitions={"succeeded": "MANIPULATION", "failed": "VISION"})
            smach.StateMachine.add("MANIPULATION", Manipulation(self), transitions={"failed": "VISION", "exit": "succeeded"})

        outcome = sm.execute()

def main():
    rclpy.init()
    node = Bringme_state()
    node.execute()
    
if __name__ == '__main__':    
    main()


class Voice(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, output_keys=["target_object"], outcomes=["succeeded", "failed"])

        self.node = node

        # Create node    
        self.logger = self.node.get_logger()

        self.cli = self.node.create_client(StringCommand, 'voice/command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):        
        self.logger.info("Start voice recognition")        
        
        self.req.command = "start"        
        result = self.send_request()        
        target_object, target_location = result.answer.split(',')
            
        userdata.target_object = target_object    
        userdata.target_location = target_location    
            
        if len(target_object) > 0 and len(target_location) > 0:    
            return "succeeded"        
        else:        
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)    
    
        while rclpy.ok():    
            rclpy.spin_once(self.node)    
            if self.future.done():    
                response = self.future.result()
                response.answer = "Bring me a cup from kitchen"
                break

        return response.answer


class Navigation(smach.State):    
    def __init__(self, node):           
        smach.State.__init__(self, input_keys=["target_object"], outcomes=["succeeded", "failed"])    

        self.node = node

        # Define logger                                                                               
        self.logger = self.node.get_logger()    
                                                
        self.cli = self.node.create_client(StringCommand, 'navigation/command')    
                                                                                   
        while not self.cli.wait_for_service(timeout_sec=1.0):                          
            self.logger.info('service not available, waiting again...')    
        self.req = StringCommand.Request()                                 
                                                                               
        self.result = None                    

    def execute(self, userdata):

        self.req.command = userdata.target_location    
        result = self.send_request()

        if result:
            return "succeeded"
        else:
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="reached":
            return True
        else:
            return False


class Vision(smach.State):    
    def __init__(self, node):    
        smach.State.__init__(self, input_keys=["target_object"], output_keys=["target_object_pos"], outcomes=["succeeded", "failed"])

        self.node = node

        # Define logger
        self.logger = self.node.get_logger()
                                                    
        self.cli = self.node.create_client(StringCommand, 'vision/command')
                                                                                   
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')
        self.req = StringCommand.Request()                                     
                                              
        self.result = None

    def execute(self, userdata):

        self.req.command = userdata.target_object   
        result = self.send_request()

        if result:
            return "succeeded"
        else:
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="detected":
            return True
        else:
            return False


class Manipulation(smach.State):    
    def __init__(self, node):    
        smach.State.__init__(self, input_keys=["target_object_pos"], outcomes=["exit", "failed"])    

        self.node = node

        # Define logger    
        self.logger = self.node.get_logger()        
        
        self.cli = self.node.create_client(StringCommand, 'vision/command')        
        
        while not self.cli.wait_for_service(timeout_sec=1.0):        
            self.logger.info('service not available, waiting again...')        
        self.req = StringCommand.Request()    
    
        self.result = None    

    def execute(self, userdata):
        self.logger.info("Try to grasp the target object")

        self.req.command = "start"    
        result = self.send_request()

        if result:
            return "exit"
        else:
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="reached":
            return True
        else:
            return False


