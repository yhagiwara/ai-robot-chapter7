# ファイル名：main_sm.py in bringme_sm
    
import rclpy
from rclpy.node import Node
import smach    
    
from ai_robot_book_interfaces.srv import StringCommand


# Bring meタスクのステートマシーンを実行するノードを定義します．
class Bringme_state(Node):
    def __init__(self):
        super().__init__("bringme_state")

    def execute(self):
        # SMACHステートマシーンを作成
        sm = smach.StateMachine(outcomes=["succeeded"])

        # コンテナにステートを追加
        with sm:
            smach.StateMachine.add("VOICE", Voice(self), transitions={"succeeded": "NAVIGATION", "failed": "VOICE"})
            smach.StateMachine.add("NAVIGATION", Navigation(self), transitions={"succeeded": "VISION", "failed": "NAVIGATION"})
            smach.StateMachine.add("VISION", Vision(self), transitions={"succeeded": "MANIPULATION", "failed": "VISION"})
            smach.StateMachine.add("MANIPULATION", Manipulation(self), transitions={"failed": "VISION", "exit": "succeeded"})

        # SMACHプランを実行
        outcome = sm.execute()
        
def main():    
    rclpy.init()    
    node = Bringme_state()    
    node.execute()
    

# 音声関連のダミーステート
class Voice(smach.State):    
    def __init__(self, node):    
        smach.State.__init__(self, output_keys=["target_object", "target_location"], outcomes=["succeeded", "failed"])
    
        # Nodeを作成しています
        self.node = node
        self.logger = self.node.get_logger()    
    
        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'voice/command')    
        while not self.cli.wait_for_service(timeout_sec=1.0):    
            self.logger.info('サービスへの接続待ちです・・・')    
        self.req = StringCommand.Request()    
    
        self.result = None

    def execute(self, userdata):        
        self.logger.info("音声認識ステートを開始します")
        
        self.req.command = "start"        
        result = self.send_request()        
            
        target_object = "cup" # find_object_name(result)    
        target_location = "kitchen" # find_location_name(result)    
        userdata.target_object = target_object    
        userdata.target_location = target_location    
            
        if len(target_object) > 0 and len(target_location) > 0:    
            return "succeeded"        
        else:        
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)    
    
        # サービスを動作させる処理
        while rclpy.ok():    
            rclpy.spin_once(self.node)    
            if self.future.done():    
                response = self.future.result()
                response.answer = "Bring me a cup from kitchen"
                break

        return response.answer


# 移動関連のダミーステート
class Navigation(smach.State):    
    def __init__(self, node):           
        smach.State.__init__(self, input_keys=["target_location"], outcomes=["succeeded", "failed"])

        # Nodeを作成しています
        self.node = node
        self.logger = self.node.get_logger()    
        
        # サービスにおけるクライアントを作成                                     
        self.cli = self.node.create_client(StringCommand, 'navigation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')    
        self.req = StringCommand.Request()                                 
                                                                               
        self.result = None

    def execute(self, userdata):
        self.logger.info("ナビゲーションステートを開始します")

        self.req.command = userdata.target_location    
        result = self.send_request()

        return result

    def send_request(self):    
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="reached":
            return True
        else:
            return False


# 物体検出関連のダミーステート
class Vision(smach.State):    
    def __init__(self, node):    
        smach.State.__init__(self, input_keys=["target_object"], output_keys=["target_object_pos"], outcomes=["succeeded", "failed"])

        # Nodeを作成しています
        self.node = node
        self.logger = self.node.get_logger()
        
        # サービスにおけるクライアントを作成                                       
        self.cli = self.node.create_client(StringCommand, 'vision/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()                                     
                                              
        self.result = None

    def execute(self, userdata):
        self.logger.info("物体認識ステートを開始します")

        self.req.command = userdata.target_object   
        result = self.send_request()
        userdata.target_object_pos = [0.12, -0.03, 0.4] # 単位は[m]

        if result:
            return "succeeded"
        else:
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="detected":
            return True
        else:
            return False


# 物体把持関連のダミーステート
class Manipulation(smach.State):    
    def __init__(self, node):    
        smach.State.__init__(self, input_keys=["target_object_pos"], outcomes=["exit", "failed"])

        # Nodeを作成しています
        self.node = node
        self.logger = self.node.get_logger()        
        
        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'vision/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):        
            self.logger.info('サービスへの接続待ちです・・・')        
        self.req = StringCommand.Request()    
    
        self.result = None

    def execute(self, userdata):
        self.logger.info("物体把持ステートを開始します")

        target_object_pos = userdata.target_object_pos
        self.req.command = "start"    
        result = self.send_request()

        if result:
            return "exit"
        else:
            return "failed"

    def send_request(self):    
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer=="reached":
            return True
        else:
            return False
