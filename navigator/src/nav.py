#!/usr/bin/env python

import rospy
import smach
import smach_ros
from states import *

from std_msgs.msg import Empty

import numpy as np

def state_machine_handler():
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Land'])

    sm_top.userdata.enableYoloLoop = True
    sm_top.userdata.normHeight = 1
    sm_top.userdata.currentAltitude = 1
    sm_top.userdata.enableDownCamLoop = False
    sm_top.userdata.targetYolo = [1, 2]
    sm_top.userdata.yawPidYolo = 0
    sm_top.userdata.pitchPidYolo = 0

    # Open the container
    with sm_top:


        sm_con = smach.Concurrence(outcomes=['outcome4','outcome5'],
                                   default_outcome='outcome4',
                                   outcome_map={'outcome5':{ 'FOO':'outcome2', 'BAR':'outcome1'}})
        with sm_con:
            sm_TakeOff = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_TakeOff:
                smach.StateMachine.add('TakeOff', TakeOff(),
                                    transitions={'FindGR':'FindGR',
                                                    'TakeOff':'TakeOff',
                                                    'Null':'Null'},
                                    remapping={'enableYoloLoop':'enableYoloLoop',
                                                'normHeight':'normHeight',
                                                'currentAltitude':'currentAltitude'})
                
                smach.StateMachine.add('FindGR', FindGR(),
                                    transitions={'RandomTraversal':'RandomTraversal',
                                                    'FocusTarget':'FocusTarget'},
                                    remapping={'normHeight':'normHeight',
                                                'currentAltitude':'currentAltitude',
                                                'enableDownCamLoop':'enableDownCamLoop',
                                                'targetYolo':'targetYolo'})

                smach.StateMachine.add('FocusTarget', FocusTarget(),
                                    transitions={'FindGR':'FindGR'},
                                    remapping={'targetYolo':'targetYolo',
                                                'yawPidYolo':'yawPidYolo',
                                                'pitchPidYolo':'pitchPidYolo'})

                smach.StateMachine.add('RandomTraversal', RandomTraversal(),
                                    transitions={'FocusTarget':'FocusTarget'})
            
            sm_CheckDownCam = smach.StateMachine(outcomes=['Obstacle', 'Null'])
            
            with sm_CheckDownCam:
                smach.StateMachine.add('CheckDownCam', CheckDownCam(),
                                    transitions={'FindGR':'FindGR',
                                                    'TakeOff':'TakeOff',
                                                    'Null':'Null'},
                                    remapping={'enableYoloLoop':'enableYoloLoop',
                                                'normHeight':'normHeight',
                                                'currentAltitude':'currentAltitude'})
            
            sm_StartInteract = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_StartInteract:


            smach.Concurrence.add('sm_TakeOff', sm_TakeOff, transitions={'Null':'Null'})
            smach.Concurrence.add('sm_CheckDownCam', sm_CheckDownCam, transitions={'Null':'Null'})
            smach.Concurrence.add('sm_StartInteract', sm_StartInteract, transitions={'Null':'Null'})
            

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()

def main():
    rospy.init_node('IARC_SM')
    state_machine_handler()

if __name__ == '__main__':
    main()
    