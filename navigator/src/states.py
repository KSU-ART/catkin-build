import rospy
import smach
import smach_ros

from std_msgs.msg import Float32, String, Bool, Int16

import numpy as np
import json
import time
import math

DEBUG = True

class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['FindGR', 'TakeOff', 'Null'], input_keys=['enableTakeOffLoop', 'normHeight', 'altitudeDeviation'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        rospy.Subscriber("/IARC/currentAltitude", Float32, callback=self.callback)
        self.altitude = 0

    def callback(self, msg):
        self.altitude = msg.data

    def execute(self, userdata):
        if userdata.enableTakeOffLoop:
            # set altitude to normal height
            self.pubTargetAltitude.publish(Float32(userdata.normHeight))
            # check current altitude
            if DEBUG:
                print(self.altitude)
            if abs(self.altitude - userdata.normHeight) < userdata.altitudeDeviation:
                # reaches target +- 0.1 meters
                return 'FindGR'
            else:
                return 'TakeOff'
        else:
            return 'Null'
        
class FindGR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RandomTraversal', 'FocusTarget'], output_keys=['enableCheckDownCamLoop', 'targetYolo'])
        rospy.Subscriber("/IARC/YOLO", String, callback=self.callback)
        XtargetYoloPub = rospy.Publisher('/IARC/YOLO/target/x', Int16, queue_size=1)
        YtargetYoloPub = rospy.Publisher('/IARC/YOLO/target/y', Int16, queue_size=1)
        self.emptyYOLO = False
        self.minYolo = None

    def callback(self, msg):
        stringData = msg.data
        if DEBUG:
            print("yolo string: ",stringData)
        data = json.loads(stringData)
        # check if yolo message is empty (aka no ground robot detected)
        if len(data) == 0:
            self.emptyYOLO = True
            if DEBUG:
                print("yolo is empty")
        else:
            # find the min(y) yolo coordinate and set to minYolo
            npData = np.array(data)
            minargs = np.argmin(npData, axis=0)
            minCoord = npData[minargs[-1]]
            self.minYolo = minCoord[1:]
            # debug
            if DEBUG:
                # print minargs
                print self.minYolo
            XtargetYoloPub.publish(Int16(self.minYolo[0]))
            YtargetYoloPub.publish(Int16(self.minYolo[1]))

    def execute(self, userdata):
        userdata.enableCheckDownCamLoop = True
        if self.emptyYOLO:
            # no ground robot detected
            return 'RandomTraversal'
        else:
            if self.minYolo != None:
                userdata.targetYolo = self.minYolo
            return 'FocusTarget'

class FocusTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['FindGR'], input_keys=['targetYolo'], output_keys=['yawPidYolo', 'pitchPidYolo'])

    def execute(self, userdata):
        if userdata.targetYolo != None:
            userdata.yawPidYolo = userdata.targetYolo[0]
            userdata.pitchPidYolo = userdata.targetYolo[1]
        else:
            print("--- Could not find targetYOLO ---")
        return 'FindGR'

###################### TODO: Needs more planing ########################
class RandomTraversal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['FocusTarget'])
        rospy.Subscriber("/IARC/currentAngle", Float32, callback=self.callback)

    def callback(self, msg):
        self.angle = msg.data

    def execute(self, userdata):
        return 'FocusTarget'


####################### Down Cam Node ###########################
class CheckDownCam(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckDownCam', 'FollowGR', 'Null'], input_keys=['enableCheckDownCamLoop'], output_keys=['enableTakeOffLoop', 'GRdist', 'GRangle'])
        rospy.Subscriber("/IARC/OrientationNet/angle", Float32, callback=self.callbackOrientation)
        rospy.Subscriber("/IARC/OrientationNet/pos/x", Int16, callback=self.callbackPosX)
        rospy.Subscriber("/IARC/OrientationNet/pos/y", Int16, callback=self.callbackPosY)
        rospy.Subscriber("/IARC/OrientationNet/detected", Bool, callback=self.callbackDetect)

        self.GRfound = False
        self.grAngle = 0
        self.posX = -1
        self.posY = -1

    def callbackDetect(self, msg):
        self.GRfound = msg.data

    def callbackOrientation(self, msg):
        self.grAngle = msg.data

    def callbackPosX(self, msg):
        self.posX = msg.data

    def callbackPosY(self, msg):
        self.posY = msg.data

    def execute(self, userdata):
        if userdata.enableCheckDownCamLoop:
            userdata.GRdist = [self.posX, self.posY]
            userdata.GRangle = self.grAngle
            if self.GRfound == False:
                userdata.enableTakeOffLoop = True
                return 'CheckDownCam'
            else:
                userdata.enableTakeOffLoop = False
                return 'FollowGR'
        else:
            return 'Null'
    
class FollowGR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckDownCam'], input_keys=['GRdist', 'GRangle', 'minGoalAngle', 'maxGoalAngle'], output_keys=['enableStartInteractLoop'])
        self.pubPitchPID = rospy.Publisher('/IARC/DownCam/PitchPID', Float32, queue_size=1)
        self.pubRollPID = rospy.Publisher('/IARC/DownCam/RollPID', Float32, queue_size=1)

    def execute(self, userdata):
        self.pubPitchPID(Float32(userdata.GRdist[1]))
        self.pubRollPID(Float32(userdata.GRdist[0]))
        if userdata.GRangle < minGoalAngle or userdata.GRangle > maxGoalAngle:
            userdata.enableStartInteractLoop = True
        return 'CheckDownCam'

############################## Interact Node ###############################
class StartInteract(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TouchDown', 'StartInteract', 'Null'], input_keys=['lowHeight', 'altitudeDeviation', 'TouchDownTimerMAX', 'enableStartInteractLoop'], output_keys=['enableCheckDownCamLoop', 'TouchDownTimer'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        rospy.Subscriber("/IARC/currentAltitude", Float32, callback=self.callback)
        self.altitude = 0

    def callback(self, msg):
        self.altitude = msg.data

    def execute(self, userdata):
        if userdata.enableStartInteractLoop:
            self.pubTargetAltitude.publish(Float32(userdata.lowHeight))
            if abs(self.altitude - userdata.lowHeight) < userdata.altitudeDeviation:
                userdata.enableCheckDownCamLoop = False
                userdata.TouchDownTimer = userdata.TouchDownTimerMAX
                return 'TouchDown'
            else:
                return 'StartInteract'
        else:
            return 'Null'

class TouchDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['AccendingCraft', 'TouchDown'], input_keys=['In_TouchDownTimer', 'groundHeight'], output_keys=['Out_TouchDownTimer'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        self.startTime = time.time()

    def execute(self, userdata):
        endTime = time.time()
        userdata.Out_TouchDownTimer = userdata.In_TouchDownTimer - (endTime - self.startTime)
        self.pubTargetAltitude.publish(Float32(userdata.groundHeight))
        if userdata.In_TouchDownTimer <= 0:
            return 'AccendingCraft'
        else:
            return 'TouchDown'

class AccendingCraft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['AccendingCraft'], input_keys=['lowHeight', 'normHeight', 'altitudeDeviation'], output_keys=['enableCheckDownCamLoop', 'enableStartInteractLoop'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        rospy.Subscriber("/IARC/currentAltitude", Float32, callback=self.callback)
        self.altitude = 0

    def callback(self, msg):
        self.altitude = msg.data

    def execute(self, userdata):
        self.pubTargetAltitude.publish(Float32(userdata.normHeight))
        if self.altitude >= userdata.lowHeight:
            userdata.enableCheckDownCamLoop = True
        if abs(self.altitude - userdata.normHeight) < userdata.altitudeDeviation:
            userdata.enableStartInteractLoop = False
        return 'AccendingCraft'

############################# Obstacle Avoidence ############################
class CheckObstacles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckObstacles', 'ObstacleAvoidence'], input_keys=['obstacleThreshDist'])
        rospy.Subscriber("/IARC/Obstacle/dist", Float32, callback=self.callbackDist)
        self.dist = 2

    def callbackDist(self, msg):
        self.dist = msg.data

    def execute(self, userdata):
        if self.dist <= userdata.obstacleThreshDist:
            return 'ObstacleAvoidence'
        else:
            return 'CheckObstacles'

class ObstacleAvoidence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckObstacles'], input_keys=['enableTakeOffLoop', 'enableCheckDownCamLoop', 'enableStartInteractLoop'])
        rospy.Subscriber("/IARC/Obstacle/angle", Float32, callback=self.callbackAngle)
        rospy.Subscriber("/IARC/Obstacle/dist", Float32, callback=self.callbackDist)
        self.angle = 0
        self.dist = 2
        self.pubPitchPID = rospy.Publisher('/IARC/Obstacle/PitchPID', Float32, queue_size=1)
        self.pubRollPID = rospy.Publisher('/IARC/Obstacle/RollPID', Float32, queue_size=1)

    def callbackDist(self, msg):
        self.dist = msg.data

    def callbackAngle(self, msg):
        self.angle = msg.data

    def execute(self, userdata):
        userdata.enableTakeOffLoop = True
        userdata.enableCheckDownCamLoop = False
        userdata.enableStartInteractLoop = False

        oppositeVec = (self.dist * -math.cos(self.angle), self.dist * -math.sin(self.angle))
        if DEBUG:
            print(oppositeVec)
        
        self.pubRollPID.publish(oppositeVec[0])
        self.pubPitchPID.publish(oppositeVec[1])

        return 'CheckObstacles'
        