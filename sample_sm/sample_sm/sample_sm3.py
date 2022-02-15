# ファイル名：sample_sm2.py
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import smach
import smach_ros


# define state Foo
class Foo(smach_ros.RosState):
    def __init__(self):
        smach_ros.RosState.__init__(self, Node("Foo"), outcomes=['outcome1','outcome2'])
        self.counter = 0
        self.logger = self.node.get_logger()

    def execute(self, userdata):
        self.logger.info('状態FOOを実行中')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach_ros.RosState):
    def __init__(self):
        smach_ros.RosState.__init__(self, Node("Bar"), outcomes=['outcome1'])
        self.logger = self.node.get_logger()
 
    def execute(self, userdata):
        self.logger.info('状態BARを実行中')
        return 'outcome1'

 
def main():
    rclpy.init()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
 
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),transitions={'outcome1':'FOO'})
 
    # Execute SMACH plan
    outcome = sm.execute()
 
 
if __name__ == '__main__':
    main()
