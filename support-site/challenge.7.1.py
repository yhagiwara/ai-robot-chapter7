# ファイル名：sample_sm.py

import rclpy
from rclpy.node import Node

import smach

import random

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

# define state Grasp
class Grasp(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.logger = _node.get_logger()

    def execute(self, userdata):
        self.logger.info('I graspped a food.')

        prob = random.random()

        if prob >= 0.5:
            return 'succeeded'

        return 'failed'

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
                transitions={'succeeded': 'GRASP', 'finished': 'end'})
            smach.StateMachine.add(
                'GRASP', Grasp(self),
                transitions={'succeeded': 'EAT', 'failed': 'GRASP'})
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


"""一時的に出力をここに記述
[ DEBUG ] : Adding state (SEARCH, <__main__.Search object at 0x7fe01c483df0>, {'succeeded': 'GRASP', 'finished': 'end'})
[ DEBUG ] : Adding state 'SEARCH' to the state machine.
[ DEBUG ] : State 'SEARCH' is missing transitions: {}
[ DEBUG ] : TRANSITIONS FOR SEARCH: {'succeeded': 'GRASP', 'finished': 'end'}
[ DEBUG ] : Adding state (GRASP, <__main__.Grasp object at 0x7fe01c483e80>, {'succeeded': 'EAT', 'failed': 'GRASP'})
[ DEBUG ] : Adding state 'GRASP' to the state machine.
[ DEBUG ] : State 'GRASP' is missing transitions: {}
[ DEBUG ] : TRANSITIONS FOR GRASP: {'succeeded': 'EAT', 'failed': 'GRASP'}
[ DEBUG ] : Adding state (EAT, <__main__.Eat object at 0x7fe01c483b80>, {'done': 'SEARCH'})
[ DEBUG ] : Adding state 'EAT' to the state machine.
[ DEBUG ] : State 'EAT' is missing transitions: {}
[ DEBUG ] : TRANSITIONS FOR EAT: {'done': 'SEARCH'}
[  INFO ] : State machine starting in initial state 'SEARCH' with userdata: 
	[]
[INFO] [1645967679.654245847] [state_machine]: I am searching.
[INFO] [1645967679.654493238] [state_machine]: Got a sweet.
[  INFO ] : State machine transitioning 'SEARCH':'succeeded'-->'GRASP'
[INFO] [1645967679.654732177] [state_machine]: I graspped a food.
[  INFO ] : State machine transitioning 'GRASP':'failed'-->'GRASP'
[INFO] [1645967679.654967337] [state_machine]: I graspped a food.
[  INFO ] : State machine transitioning 'GRASP':'failed'-->'GRASP'
[INFO] [1645967679.655198608] [state_machine]: I graspped a food.
[  INFO ] : State machine transitioning 'GRASP':'failed'-->'GRASP'
[INFO] [1645967679.655429608] [state_machine]: I graspped a food.
[  INFO ] : State machine transitioning 'GRASP':'succeeded'-->'EAT'
[INFO] [1645967679.655659035] [state_machine]: I am eating.
[  INFO ] : State machine transitioning 'EAT':'done'-->'SEARCH'
[INFO] [1645967679.655911782] [state_machine]: I am searching.
[INFO] [1645967679.656131619] [state_machine]: Got a sweet.
[  INFO ] : State machine transitioning 'SEARCH':'succeeded'-->'GRASP'
[INFO] [1645967679.656376124] [state_machine]: I graspped a food.
[  INFO ] : State machine transitioning 'GRASP':'succeeded'-->'EAT'
[INFO] [1645967679.656607392] [state_machine]: I am eating.
[  INFO ] : State machine transitioning 'EAT':'done'-->'SEARCH'
[INFO] [1645967679.656835255] [state_machine]: I am searching.
[INFO] [1645967679.657070629] [state_machine]: Got a sweet.
[  INFO ] : State machine transitioning 'SEARCH':'succeeded'-->'GRASP'
[INFO] [1645967679.657318254] [state_machine]: I graspped a food.
[  INFO ] : State machine transitioning 'GRASP':'succeeded'-->'EAT'
[INFO] [1645967679.657550730] [state_machine]: I am eating.
[  INFO ] : State machine transitioning 'EAT':'done'-->'SEARCH'
[INFO] [1645967679.657780871] [state_machine]: I am searching.
[INFO] [1645967679.658012732] [state_machine]: I am full.
[  INFO ] : State machine terminating 'SEARCH':'finished':'end'
[INFO] [1645967679.658260258] [state_machine]: outcom: end
"""
