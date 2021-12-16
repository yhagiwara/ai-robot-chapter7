#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import smach

from time import sleep
from random import random

from geometry_msgs.msg import Pose


def main():
    rclpy.init()

    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add("VoiceState", VOICE_STATE(), transitions={"succeeded": "NavigationState", "failed": "VoiceState"})
        smach.StateMachine.add("NavigationState", NAVIGATION_STATE(), transitions={"succeeded": "VisionState", "failed": "VoiceState"})
        smach.StateMachine.add("VisionState", VISION_STATE(), transitions={"succeeded": "ManipulationState", "failed": "VoiceState"})
        smach.StateMachine.add("ManipulationState", MANIPULATION_STATE(), transitions={"failed": "VisionState", "exit": "succeeded"})

    outcome = sm_top.execute()

class VOICE_STATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, output_keys=["target_object"], outcomes=["succeeded", "failed"])

        # Define logger
        self.node = Node("VoiceState")
        self.logger = self.node.get_logger()

    def execute(self, userdata):
        self.logger.info("Start voice recognition")
        userdata.target_object = "Bottle" # 音声認識による結果をtarget_objectに入力

        prob = random()

        sleep(1.0)
    
        if prob > 0.3:
            self.logger.info("Succeeded voice recognition")
            return "succeeded"
        else:
            self.logger.info("Failed voice recognition and retry to recognize")
            return "failed"


class NAVIGATION_STATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=["target_object"], outcomes=["succeeded", "failed"])

        # Define logger
        self.node = Node("NavigationState")
        self.logger = self.node.get_logger()

    def execute(self, userdata):

        prob = random()

        sleep(1.0)

        if userdata.target_object == "Bottle":
        
            if prob > 0.4:
                self.logger.info("Moved to x:1.0, y:1.0")
                return "succeeded"
            else:
                self.logger.info("Failed to move")
                return "failed"

        return "failed"


class VISION_STATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=["target_object"], output_keys=["target_object_pos"], outcomes=["succeeded", "failed"])

        # Define logger
        self.node = Node("VisionState")
        self.logger = self.node.get_logger()

    def execute(self, userdata):
        sleep(1.0)

        target_object = userdata.target_object

        result, target_object_pos = self.detect_object_using_YOLO(target_object)

        if result:
            userdata.target_object_pos = target_object_pos
            return "succeeded"
        else:
            return "failed"

    def detect_object_using_YOLO(self, target_object):
        prob = random()

        if prob > 0.2:
            self.logger.info(f"Detected {target_object}")
            return True, (1.1, 1.2, 0.5)
        else:
            self.logger.info(f"Not found {target_object}")
            return False, ()


class MANIPULATION_STATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=["target_object_pos"], outcomes=["exit", "failed"])

        # Define logger
        self.node = Node("ManipulationState")
        self.logger = self.node.get_logger()

    def execute(self, userdata):
        self.logger.info(f"Try to grasp the target object")

        sleep(1.0)

        target_object_pos = userdata.target_object_pos

        result = self.manipulate(target_object_pos)

        if result:
            self.logger.info("Bring me task is completed")
            return "exit"
        else:
            self.logger.info("Retry to find the target object")
            return "failed"

    def manipulate(self, target_object_pos):
        prob = random()

        pos = target_object_pos
        self.logger.info(f"Now reaching the target object x:{pos[0]} y:{pos[1]} z:{pos[2]}")

        if prob > 0.3:
            self.logger.info("Succeeded to grasp the target object")
            return True
        else:
            self.logger.info("Failed to grasp the target object")
            return False



if __name__ == '__main__':
    main()
