#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from ai_robot_book_interfaces.srv import StringCommand

import smach


class SampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, output_keys=["target_object"], outcomes=["exit"])

        # Create node
        self.node = Node("sample_state")
        self.logger = self.node.get_logger()

        self.cli = self.node.create_client(StringCommand, 'command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):

        self.send_request()

        return "exit"

    def send_request(self):
        self.req.command = "check"
        self.future = self.cli.call_async(self.req)
        print(self.future)


def main():
    rclpy.init()

    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add("SampleState", SampleState(), transitions={"exit": "succeeded"})

    outcome = sm_top.execute()


if __name__ == "__main__":
    main()
