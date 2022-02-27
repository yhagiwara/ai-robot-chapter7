# ファイル名：sample_sm.py
    
import rclpy
from rclpy.node import Node
import smach


# define state Search
class Search(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['succeeded','finished'])
        self.counter = 0
        self.logger = _node.get_logger()

    def execute(self, userdata):
        self.logger.info('I am searching.')
        if self.counter < 3:
            self.logger.info('Got a sweet.')
            self.counter += 1
            return 'succeeded'
        else:
            self.logger.info('I am full.')
            return 'finished'

# define state Eat
class Eat(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['done'])
        self.logger = _node.get_logger()

    def execute(self, userdata):
        self.logger.info('I am eating.')
        return 'done'


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

    def execute(self):
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['end'])
        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add(
                'SEARCH', Search(self),
                transitions={'succeeded': 'EAT', 'finished': 'end'})
            smach.StateMachine.add(
                'EAT', Eat(self),
                transitions={'done': 'SEARCH'})

        # Execute SMACH plan
        outcome = sm.execute()
        self.get_logger().info(f'outcom: {outcome}')

def main():
    rclpy.init()
    node = StateMachine()
    node.execute()

if __name__ == '__main__':
    main()
