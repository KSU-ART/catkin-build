#!/usr/bin/env python

import rospy
import smach
import smach_ros
from states import *

from std_msgs.msg import Empty

import numpy as np

def state_machine_handler():
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Done'])

    sm_top.userdata.enableTakeOffLoop = True
    sm_top.userdata.enableCheckDownCamLoop = False
    sm_top.userdata.enableStartInteractLoop = False

    sm_top.userdata.normHeight = 1
    sm_top.userdata.altitudeDeviation = 0.1
    sm_top.userdata.targetYolo = [1, 2]
    sm_top.userdata.yawPidYolo = 0
    sm_top.userdata.pitchPidYolo = 0

    sm_top.userdata.GRdist = 0
    sm_top.userdata.GRangle = 0
    sm_top.userdata.minGoalAngle = 20
    sm_top.userdata.maxGoalAngle = 20

    sm_top.userdata.lowHeight = 0.2
    sm_top.userdata.TouchDownTimerMAX = 1
    sm_top.userdata.TouchDownTimer = 0
    sm_top.userdata.groundHeight = 0

    sm_top.userdata.obstacleThreshDist = 1.5   

    # Open the container
    with sm_top:

        sm_con = smach.Concurrence(outcomes=['Null','Obstacle'],
                                   default_outcome='Null',
                                   outcome_map={'Obstacle':{ 'sm_TakeOff':'Obstacle', 'sm_CheckDownCam':'Obstacle', 'sm_StartInteract':'Obstacle'}})
        with sm_con:
            sm_TakeOff = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_TakeOff:
                sm_TakeOff.userdata.enableTakeOffLoop = sm_top.userdata.enableTakeOffLoop
                sm_TakeOff.userdata.normHeight = sm_top.userdata.normHeight
                sm_TakeOff.userdata.altitudeDeviation = sm_top.userdata.altitudeDeviation
                sm_TakeOff.userdata.enableCheckDownCamLoop = sm_top.userdata.enableCheckDownCamLoop
                sm_TakeOff.userdata.targetYolo = sm_top.userdata.targetYolo
                sm_TakeOff.userdata.yawPidYolo = sm_top.userdata.yawPidYolo
                sm_TakeOff.userdata.pitchPidYolo = sm_top.userdata.pitchPidYolo

                smach.StateMachine.add('TakeOff', TakeOff(),
                                    transitions={'FindGR':'FindGR',
                                                 'TakeOff':'TakeOff',
                                                 'Null':'Null'},
                                    remapping={'enableTakeOffLoop':'enableTakeOffLoop',
                                               'normHeight':'normHeight',
                                               'altitudeDeviation':'altitudeDeviation'})
                
                smach.StateMachine.add('FindGR', FindGR(),
                                    transitions={'RandomTraversal':'RandomTraversal',
                                                 'FocusTarget':'FocusTarget'},
                                    remapping={'enableCheckDownCamLoop':'enableCheckDownCamLoop',
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
                sm_CheckDownCam.userdata.enableCheckDownCamLoop = sm_top.userdata.enableCheckDownCamLoop
                sm_CheckDownCam.userdata.enableTakeOffLoop = sm_top.userdata.enableTakeOffLoop
                sm_CheckDownCam.userdata.GRdist = sm_top.userdata.GRdist
                sm_CheckDownCam.userdata.GRangle = sm_top.userdata.GRangle
                sm_CheckDownCam.userdata.minGoalAngle = sm_top.userdata.minGoalAngle
                sm_CheckDownCam.userdata.maxGoalAngle = sm_top.userdata.maxGoalAngle
                sm_CheckDownCam.userdata.enableStartInteractLoop = sm_top.userdata.enableStartInteractLoop

                smach.StateMachine.add('CheckDownCam', CheckDownCam(),
                                    transitions={'CheckDownCam':'CheckDownCam',
                                                 'FollowGR':'FollowGR',
                                                 'Null':'Null'},
                                    remapping={'enableCheckDownCamLoop':'enableCheckDownCamLoop',
                                               'enableTakeOffLoop':'enableTakeOffLoop',
                                               'GRdist':'GRdist',
                                               'GRangle':'GRangle'})

                smach.StateMachine.add('FollowGR', FollowGR(),
                                    transitions={'CheckDownCam':'CheckDownCam'},
                                    remapping={'GRdist':'GRdist',
                                               'GRangle':'GRangle',
                                               'minGoalAngle':'minGoalAngle',
                                               'maxGoalAngle':'maxGoalAngle',
                                               'enableStartInteractLoop':'enableStartInteractLoop'})
            
            sm_StartInteract = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_StartInteract:
                sm_StartInteract.userdata.enableStartInteractLoop = sm_top.userdata.enableStartInteractLoop
                sm_StartInteract.userdata.lowHeight = sm_top.userdata.lowHeight
                sm_StartInteract.userdata.altitudeDeviation = sm_top.userdata.altitudeDeviation
                sm_StartInteract.userdata.TouchDownTimerMAX = sm_top.userdata.TouchDownTimerMAX
                sm_StartInteract.userdata.TouchDownTimer = sm_top.userdata.TouchDownTimer
                sm_StartInteract.userdata.groundHeight = sm_top.userdata.groundHeight
                sm_StartInteract.userdata.normHeight = sm_top.userdata.normHeight             

                smach.StateMachine.add('StartInteract', StartInteract(),
                                    transitions={'TouchDown':'TouchDown',
                                                 'StartInteract':'StartInteract',
                                                 'Null':'Null'},
                                    remapping={'lowHeight':'lowHeight',
                                               'altitudeDeviation':'altitudeDeviation',
                                               'TouchDownTimerMAX':'TouchDownTimerMAX',
                                               'enableStartInteractLoop':'enableStartInteractLoop',
                                               'enableCheckDownCamLoop':'enableCheckDownCamLoop',
                                               'TouchDownTimer':'TouchDownTimer'})

                smach.StateMachine.add('TouchDown', TouchDown(),
                                    transitions={'AccendingCraft':'AccendingCraft',
                                                 'TouchDown':'TouchDown'},
                                    remapping={'In_TouchDownTimer':'TouchDownTimer',
                                               'groundHeight':'groundHeight',
                                               'Out_TouchDownTimer':'TouchDownTimer'})

                smach.StateMachine.add('AccendingCraft', AccendingCraft(),
                                    transitions={'AccendingCraft':'AccendingCraft'},
                                    remapping={'lowHeight':'lowHeight',
                                               'normHeight':'normHeight',
                                               'altitudeDeviation':'altitudeDeviation',
                                               'enableCheckDownCamLoop':'enableCheckDownCamLoop',
                                               'enableStartInteractLoop':'enableStartInteractLoop'})

            sm_CheckObstacles = smach.StateMachine(outcomes=['Null'])
            
            with sm_CheckObstacles:
                sm_CheckObstacles.userdata.obstacleThreshDist = sm_top.userdata.obstacleThreshDist
                sm_CheckObstacles.userdata.enableTakeOffLoop = sm_top.userdata.enableTakeOffLoop
                sm_CheckObstacles.userdata.enableCheckDownCamLoop = sm_top.userdata.enableCheckDownCamLoop
                sm_CheckObstacles.userdata.enableStartInteractLoop = sm_top.userdata.enableStartInteractLoop

                smach.StateMachine.add('CheckObstacles', CheckObstacles(),
                                    transitions={'CheckObstacles':'CheckObstacles',
                                                 'ObstacleAvoidence':'ObstacleAvoidence'},
                                    remapping={'obstacleThreshDist':'obstacleThreshDist'})
                
                smach.StateMachine.add('ObstacleAvoidence', ObstacleAvoidence(),
                                    transitions={'CheckObstacles':'CheckObstacles'},
                                    remapping={'enableTakeOffLoop':'enableTakeOffLoop',
                                               'enableCheckDownCamLoop':'enableCheckDownCamLoop',
                                               'enableStartInteractLoop':'enableStartInteractLoop',
                                               'obstacleThreshDist':'obstacleThreshDist'})

            smach.Concurrence.add('sm_TakeOff', sm_TakeOff)
            smach.Concurrence.add('sm_CheckDownCam', sm_CheckDownCam)
            smach.Concurrence.add('sm_StartInteract', sm_StartInteract)
            smach.Concurrence.add('sm_CheckObstacles', sm_CheckObstacles)
        
        smach.StateMachine.add('CON', sm_con,
                               transitions={'Null':'CON',
                                            'Obstacle':'Done'})

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
    