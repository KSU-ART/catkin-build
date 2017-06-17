import rospy
import smach
import smach_ros

from std_msgs.msg import Float32, String, Bool

import numpy as np
import json

class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['FindGR', 'TakeOff', 'Null'], input_keys=['enableYoloLoop', 'normHeight', 'currentAltitude'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)

    def execute(self, userdata):
        if userdata.enableYoloLoop:
            # set altitude to normal height
            self.pubTargetAltitude.publish(Float32(userdata.normHeight))
            # check current altitude
            if abs(userdata.currentAltitude - userdata.normHeight) < 0.1:
                # reaches target +- 0.1 meters
                return 'FindGR'
            else:
                return 'TakeOff'
        else:
            return 'Null'
        
class FindGR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RandomTraversal', 'FocusTarget'], input_keys=['normHeight', 'currentAltitude'], output_keys=['enableDownCamLoop', 'targetYolo'])
        rospy.Subscriber("/IARC/YOLO", String, callback=self.callback)
        self.emptyYOLO = False
        self.minYolo = None

    def callback(self, msg):
        stringData = msg.data
        data = json.loads(stringData)
        # check if yolo message is empty (aka no ground robot detected)
        if len(data) == 0:
            self.emptyYOLO = True
        else:
            # find the min(y) yolo coordinate and set to minYolo
            npData = np.array(data)
            minargs = np.argmin(npData, axis=1)
            minCoord = npData[minargs[-1]]
            self.minYolo = minCoord[1:]
            # debug
            print self.minYolo

    def execute(self, userdata):
        userdata.enableDownCamLoop = True
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



class CheckDownCam(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckDownCam', 'FollowGR'], output_keys=['enableYoloLoop', 'GRdist', 'GRangle'])
        rospy.Subscriber("/IARC/OrientationNet/angle", Float32, callback=self.callbackOrientation)
        rospy.Subscriber("/IARC/OrientationNet/pos/x", Float32, callback=self.callbackPosX)
        rospy.Subscriber("/IARC/OrientationNet/pos/y", Float32, callback=self.callbackPosY)
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
        userdata.GRdist = [self.posX, self.posY]
        userdata.GRangle = self.grAngle
        if self.GRfound == False:
            userdata.enableYoloLoop = True
            return 'CheckDownCam'
        else:
            userdata.enableYoloLoop = False
            return 'FollowGR'
    
class FollowGR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckDownCam'], input_keys=['GRdist', 'GRangle', 'minGoalAngle', 'maxGoalAngle'])
        self.pubPitchPID = rospy.Publisher('/IARC/DownCam/PitchPID', Float32, queue_size=1)
        self.pubRollPID = rospy.Publisher('/IARC/DownCam/RollPID', Float32, queue_size=1)

    def execute(self, userdata):
        self.pubPitchPID(Float32(userdata.GRdist[1]))
        self.pubRollPID(Float32(userdata.GRdist[0]))
        if userdata.GRangle < minGoalAngle or userdata.GRangle > minGoalAngle:
            
