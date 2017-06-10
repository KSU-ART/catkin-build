#!/usr/bin/env python

import rospy
import smach
import smach_ros
from states import *

def state_machine_handler():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()

def main():
    rospy.init_node('IARC_SM')
    state_machine_handler()


if __name__ == '__main__':
    main()