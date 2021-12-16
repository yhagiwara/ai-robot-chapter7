#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import smach


class SampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, output_keys=["target_object"], outcomes=["exit"])

        # Create node
        self.node = Node("sample_state")

        # Create publisher and subscription
        self.sample_state_publisher = self.node.create_publisher(String, "/sample/request", 1)
        self.sample_state_subscriber = self.node.create_subscription(String, "/sample/response", self.sample_callback, 1)

        self.result = None

    def execute(self, userdata):
        msg = String()
        msg.data = "Hello"

        # Loop until message is recieved
        while self.result is None:
            self.sample_state_publisher.publish(msg)
            # Below program is important
            # Node about to spin at once
            rclpy.spin_once(self.node, timeout_sec=0.1)

        print(self.result)

        return "exit"

    def sample_callback(self, msg):
        self.result = msg


def main():
    rclpy.init()

    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add("SampleState", SampleState(), transitions={"exit": "succeeded"})

    outcome = sm_top.execute()


if __name__ == "__main__":
    main()
