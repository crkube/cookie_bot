#!/usr/bin/env python

import rospy
import smach
import smach_ros
from time import sleep

# The donut eating robot version 2
# this version eats only 50 donuts

# define state EatDonut
class EatDonut(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_full', 'full'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state EAT_DONUT')

        if self.counter < 100:
            self.counter += 1
            sleep(2)
            return 'not_full'
        else:
            self.counter = 0
            return 'full'


# define state GetDonut
class GetDonut(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hungry'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET_DONUT')
        sleep(2)
        return 'hungry'




# main
def main():
    rospy.init_node('donut_botSM')

    # Create a SMACH state machine
    sm_eat = smach.StateMachine(outcomes=['poop'])

    # Open the container
    with sm_eat:
        # Add states to the container
        smach.StateMachine.add('GET_DONUT', GetDonut(),
                            transitions={'hungry':'EAT_DONUT'})
        smach.StateMachine.add('EAT_DONUT', EatDonut(),
                            transitions={'not_full':'GET_DONUT',
                                'full':'poop'})


    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('EATING_server', sm_eat, 'DONUT_BOTSM')
    sis.start()


    # Execute SMACH plan
    outcome = sm_eat.execute()

    # wait for ctrl-c to stop the machine
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
