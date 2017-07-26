#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
from states import *
import threading
from multiprocessing.pool import ThreadPool

from std_msgs.msg import Empty

def state_machine_handler():
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Done'])

    sm_top.userdata.normHeight = 1
    sm_top.userdata.altitudeDeviation = 0.1
    sm_top.userdata.targetYolo = [1, 2]

    sm_top.userdata.GRdist = 0
    sm_top.userdata.GRangle = 0
    sm_top.userdata.maxGoalAngle = 20 * math.pi /180
    sm_top.userdata.randomTraversalAngleThresh = 20 * math.pi /180
    sm_top.userdata.imageWidth = 640
    sm_top.userdata.imageHeight = 480

    sm_top.userdata.lowHeight = 0.2
    sm_top.userdata.TouchDownTimerMAX = 1
    sm_top.userdata.TouchDownTimer = 0
    sm_top.userdata.groundHeight = 0

    sm_top.userdata.obstacleThreshDist = 1.5

    sm_top.userdata.EdgeDetectTimerMAX = 5
    sm_top.userdata.EdgeDetectTimer = 0

    # Open the container
    with sm_top:

        sm_con = smach.Concurrence(outcomes=['Null','Obstacle'],
                                   default_outcome='Null',
                                   outcome_map={'Obstacle':{ 'sm_TakeOff':'Obstacle', 'sm_CheckDownCam':'Obstacle', 'sm_StartInteract':'Obstacle'}})
        with sm_con:
            sm_TakeOff = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_TakeOff:
                sm_TakeOff.userdata.normHeight = sm_top.userdata.normHeight
                sm_TakeOff.userdata.altitudeDeviation = sm_top.userdata.altitudeDeviation
                sm_TakeOff.userdata.targetYolo = sm_top.userdata.targetYolo
                sm_TakeOff.userdata.randomTraversalAngleThresh = sm_top.userdata.randomTraversalAngleThresh
                sm_TakeOff.userdata.imageWidth = sm_top.userdata.imageWidth
                sm_TakeOff.userdata.imageHeight = sm_top.userdata.imageHeight

                smach.StateMachine.add('TakeOff', TakeOff(),
                                    transitions={'FindGR':'FindGR',
                                                 'TakeOff':'TakeOff'},
                                    remapping={'normHeight':'normHeight',
                                               'altitudeDeviation':'altitudeDeviation'})

                smach.StateMachine.add('FindGR', FindGR(),
                                    transitions={'RandomTraversal':'RandomTraversal',
                                                 'FindGR':'FindGR',
                                                 'TakeOff':'TakeOff'},
                                    remapping={'targetYolo':'targetYolo'})

                smach.StateMachine.add('RandomTraversal', RandomTraversal(),
                                    transitions={'RandomTraversal':'RandomTraversal',
                                                 'FindGR':'FindGR',
                                                 'TakeOff':'TakeOff'},
                                    remapping={'randomTraversalAngleThresh':'randomTraversalAngleThresh'})

            sm_CheckDownCam = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_CheckDownCam:
                sm_CheckDownCam.userdata.GRdist = sm_top.userdata.GRdist
                sm_CheckDownCam.userdata.GRangle = sm_top.userdata.GRangle
                sm_CheckDownCam.userdata.maxGoalAngle = sm_top.userdata.maxGoalAngle

                smach.StateMachine.add('CheckDownCam', CheckDownCam(),
                                    transitions={'CheckDownCam':'CheckDownCam',
                                                 'FollowGR':'FollowGR'},
                                    remapping={'GRdist':'GRdist',
                                               'GRangle':'GRangle'})

                smach.StateMachine.add('FollowGR', FollowGR(),
                                    transitions={'CheckDownCam':'CheckDownCam'},
                                    remapping={'GRdist':'GRdist',
                                               'GRangle':'GRangle',
                                               'maxGoalAngle':'maxGoalAngle'})

            sm_StartInteract = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_StartInteract:
                sm_StartInteract.userdata.lowHeight = sm_top.userdata.lowHeight
                sm_StartInteract.userdata.altitudeDeviation = sm_top.userdata.altitudeDeviation
                sm_StartInteract.userdata.TouchDownTimerMAX = sm_top.userdata.TouchDownTimerMAX
                sm_StartInteract.userdata.TouchDownTimer = sm_top.userdata.TouchDownTimer
                sm_StartInteract.userdata.groundHeight = sm_top.userdata.groundHeight
                sm_StartInteract.userdata.normHeight = sm_top.userdata.normHeight

                smach.StateMachine.add('StartInteract', StartInteract(),
                                    transitions={'TouchDown':'TouchDown',
                                                 'StartInteract':'StartInteract'},
                                    remapping={'lowHeight':'lowHeight',
                                               'altitudeDeviation':'altitudeDeviation',
                                               'TouchDownTimerMAX':'TouchDownTimerMAX',
                                               'TouchDownTimer':'TouchDownTimer'})

                smach.StateMachine.add('TouchDown', TouchDown(),
                                    transitions={'AccendingCraft':'AccendingCraft',
                                                 'TouchDown':'TouchDown',
                                                 'StartInteract':'StartInteract'},
                                    remapping={'TouchDownTimer':'TouchDownTimer',
                                               'groundHeight':'groundHeight'})

                smach.StateMachine.add('AccendingCraft', AccendingCraft(),
                                    transitions={'AccendingCraft':'AccendingCraft',
                                                 'StartInteract':'StartInteract'},
                                    remapping={'lowHeight':'lowHeight',
                                               'normHeight':'normHeight',
                                               'altitudeDeviation':'altitudeDeviation'})

            sm_CheckObstacles = smach.StateMachine(outcomes=['Null'])

            with sm_CheckObstacles:
                sm_CheckObstacles.userdata.normHeight = sm_top.userdata.normHeight
                sm_CheckObstacles.userdata.obstacleThreshDist = sm_top.userdata.obstacleThreshDist

                smach.StateMachine.add('CheckObstacles', CheckObstacles(),
                                    transitions={'CheckObstacles':'CheckObstacles',
                                                 'ObstacleAvoidence':'ObstacleAvoidence'},
                                    remapping={'obstacleThreshDist':'obstacleThreshDist'})

                smach.StateMachine.add('ObstacleAvoidence', ObstacleAvoidence(),
                                    transitions={'CheckObstacles':'CheckObstacles'},
                                    remapping={'normHeight':'normHeight',
                                               'obstacleThreshDist':'obstacleThreshDist'})

            sm_EdgeDetect = smach.StateMachine(outcomes=['Obstacle', 'Null'])

            with sm_EdgeDetect:
                sm_EdgeDetect.userdata.normHeight = sm_top.userdata.normHeight
                sm_EdgeDetect.userdata.EdgeDetectTimer = sm_top.userdata.EdgeDetectTimer
                sm_EdgeDetect.userdata.EdgeDetectTimerMAX = sm_top.userdata.EdgeDetectTimerMAX

                smach.StateMachine.add('CheckEdges', CheckEdges(),
                                    transitions={'CheckEdges':'CheckEdges',
                                                 'EdgeTimer':'EdgeTimer'},
                                    remapping={'EdgeDetectTimerMAX':'EdgeDetectTimerMAX',
                                               'EdgeDetectTimer':'EdgeDetectTimer'})

                smach.StateMachine.add('EdgeTimer', EdgeTimer(),
                                    transitions={'CheckEdges':'CheckEdges',
                                                 'EdgeTimer':'EdgeTimer',
                                                 'TowardsArena':'TowardsArena'},
                                    remapping={'EdgeDetectTimer':'EdgeDetectTimer'})

                smach.StateMachine.add('TowardsArena', TowardsArena(),
                                    transitions={'CheckEdges':'CheckEdges',
                                                 'TowardsArena':'TowardsArena'},
                                    remapping={'normHeight':'normHeight'})

            smach.Concurrence.add('sm_TakeOff', sm_TakeOff)
            smach.Concurrence.add('sm_CheckDownCam', sm_CheckDownCam)
            smach.Concurrence.add('sm_StartInteract', sm_StartInteract)
            smach.Concurrence.add('sm_CheckObstacles', sm_CheckObstacles)
            smach.Concurrence.add('sm_EdgeDetect', sm_EdgeDetect)

        smach.StateMachine.add('CON', sm_con,
                               transitions={'Null':'CON',
                                            'Obstacle':'Done'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()

    # sm_top.execute()
    pool = ThreadPool(processes=5)
    async_result = pool.apply_async(sm_top.execute)

    rospy.spin()

    sis.stop()

def main():
    rospy.init_node('IARC_SM')
    state_machine_handler()

if __name__ == '__main__':
    main()
    
