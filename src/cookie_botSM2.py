#!/usr/bin/env python

import rospy
import smach
import smach_ros
from time import sleep

# define state EatCookie
class EatCookie(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_full', 'full'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state EAT_COOKIE')

        if self.counter < 10:
            self.counter += 1
            sleep(2)
            return 'not_full'
        else:
            self.counter = 0
            return 'full'


# define state GetCookie
class GetCookie(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hungry'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET_COOKIE')
        sleep(2)
        return 'hungry'


# define state Pooping
class Pooping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tired','done'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state POOPING')
        sleep(10)
        if self.counter < 10:
            self.counter += 1
            return 'tired'
        else:
            return 'done'       # exit FSM


# define state Sleeping
class Sleeping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_tired'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SLEEPING')
        sleep(20)
        return 'not_tired'




# main
def main():
    rospy.init_node('cookie_botSM')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm_top:
        # Add states to the top container
        smach.StateMachine.add('SLEEPING', Sleeping(),
                    transitions={'not_tired':'EATING'})
        smach.StateMachine.add('POOPING', Pooping(),
                    transitions={'tired':'SLEEPING',
                        'done':'outcome5'})         # exit

        # Create a SMACH state machine
        sm_eat = smach.StateMachine(outcomes=['poop'])

        # Open the sub container
        with sm_eat:
            # Add states to the sub container
            smach.StateMachine.add('GET_COOKIE', GetCookie(),
                            transitions={'hungry':'EAT_COOKIE'})
            smach.StateMachine.add('EAT_COOKIE', EatCookie(),
                            transitions={'not_full':'GET_COOKIE',
                                'full':'poop'})

        smach.StateMachine.add('EATING', sm_eat,
                    transitions={'poop':'POOPING'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('cookie_server', sm_top, 'COOKIE_BOTSM2')
    sis.start()


    # Execute SMACH plan
    outcome = sm_top.execute()

    # wait for ctrl-c to stop the machine
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
