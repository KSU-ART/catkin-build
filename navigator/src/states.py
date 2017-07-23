import rospy
import smach
import smach_ros

from std_msgs.msg import Float32, String, Bool, Int16

import numpy as np
import json
import time
import math
import random

DEBUG = False

class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['FindGR', 'TakeOff'], input_keys=['normHeight', 'altitudeDeviation'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        rospy.Subscriber("/IARC/currentAltitude", Float32, callback=self.callback)
        self.altitude = 0
        
        rospy.Subscriber("/IARC/states/enableTakeOffLoop", Bool, callback=self.enableTakeOffLoop_cb)
        self.enableTakeOffLoop = True

    def callback(self, msg):
        self.altitude = msg.data

    def enableTakeOffLoop_cb(self, msg):
        self.enableTakeOffLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.enableTakeOffLoop:
            # set altitude to normal height
            self.pubTargetAltitude.publish(Float32(userdata.normHeight))
            # check current altitude
            if DEBUG:
                print("Altitude:", self.altitude)
            if abs(self.altitude - userdata.normHeight) < userdata.altitudeDeviation:
                # reaches target +- 0.1 meters
                return 'FindGR'
            else:
                return 'TakeOff'
        else:
            return 'TakeOff'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")
        
class FindGR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RandomTraversal', 'FindGR', 'TakeOff'], output_keys=['targetYolo', 'imageWidth', 'imageHeight'])
        rospy.Subscriber("/yolo/first/boxes", String, callback=self.callback)
        self.XtargetYoloPub = rospy.Publisher('/IARC/YOLO/target/x', Int16, queue_size=1)
        self.YtargetYoloPub = rospy.Publisher('/IARC/YOLO/target/y', Int16, queue_size=1)
        self.emptyYOLO = False
        self.minYolo = None
        
        self.enableCheckDownCamLoop_pub = rospy.Publisher('/IARC/states/enableCheckDownCamLoop', Bool, queue_size=1)

        rospy.Subscriber("/IARC/states/enableTakeOffLoop", Bool, callback=self.enableTakeOffLoop_cb)
        self.enableTakeOffLoop = True

    def callback(self, msg):
        stringData = msg.data
        if DEBUG:
            print("yolo string:",stringData)
        data = json.loads(stringData)
        # check if yolo message is empty (aka no ground robot detected)
        if len(data) == 0:
            self.emptyYOLO = True
            if DEBUG:
                print("yolo is empty")
        else:
            self.emptyYOLO = False
            ## TODO: unit test this
            # find the min(y) yolo coordinate and set to minYolo
            npData = np.array(data)
            minargs = np.argmax(npData[npData[:,0] < 2], axis=0)
            # print("minargs", npData[npData[:,0] < 2])
            minCoord = npData[minargs[2]] 
            self.minYolo = minCoord[1:3]
            # debug
            if DEBUG:
                # print minargs
                print("minYolo:", self.minYolo)
            self.XtargetYoloPub.publish(Int16(self.minYolo[0] * 640))
            self.YtargetYoloPub.publish(Int16(self.minYolo[1] * 480))

        
    def enableTakeOffLoop_cb(self, msg):
        self.enableTakeOffLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        self.enableCheckDownCamLoop_pub.publish(Bool(True))

        if not self.enableTakeOffLoop:
            return 'TakeOff'

        print("emptyYOLO:", self.emptyYOLO)

        if self.emptyYOLO:
            # no ground robot detected
            print("going to Random Traversal")
            return 'RandomTraversal'
        else:
            if self.minYolo != None:
                userdata.targetYolo = self.minYolo
            return 'FindGR'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class RandomTraversal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RandomTraversal', 'FindGR', 'TakeOff'], input_keys=['randomTraversalAngleThresh'])
        rospy.Subscriber("/IARC/currentAngle", Float32, callback=self.callback)
        self.YawPIDrt = rospy.Publisher('/IARC/randomTraversal/deltaAngle', Float32, queue_size=1)
        self.targetAngle = random.uniform(-math.pi, math.pi)
        self.angle = 0

        rospy.Subscriber("/IARC/states/enableTakeOffLoop", Bool, callback=self.enableTakeOffLoop_cb)
        self.enableTakeOffLoop = True

    def callback(self, msg):
        self.angle = msg.data

    def enableTakeOffLoop_cb(self, msg):
        self.enableTakeOffLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if not self.enableTakeOffLoop:
            return 'TakeOff'

        if abs(self.angle - self.targetAngle) < userdata.randomTraversalAngleThresh:
            self.targetAngle = random.uniform(-math.pi, math.pi)
            return 'FindGR'
        else:
            self.YawPIDrt.publish(Float32(self.angle - self.targetAngle))
            return 'RandomTraversal'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")


####################### Down Cam Node ###########################
class CheckDownCam(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckDownCam', 'FollowGR'], output_keys=['GRdist', 'GRangle'])
        rospy.Subscriber("/IARC/OrientationNet/angle", Float32, callback=self.callbackOrientation)
        rospy.Subscriber("/IARC/OrientationNet/pos/x", Int16, callback=self.callbackPosX)
        rospy.Subscriber("/IARC/OrientationNet/pos/y", Int16, callback=self.callbackPosY)
        rospy.Subscriber("/IARC/OrientationNet/detected", Bool, callback=self.callbackDetect)

        self.GRfound = False
        self.grAngle = 0
        self.posX = -1
        self.posY = -1

        rospy.Subscriber("/IARC/states/enableCheckDownCamLoop", Bool, callback=self.enableCheckDownCamLoop_cb)
        self.enableCheckDownCamLoop = False

        self.enableTakeOffLoop_pub = rospy.Publisher('/IARC/states/enableTakeOffLoop', Bool, queue_size=1)

    def callbackDetect(self, msg):
        self.GRfound = msg.data

    def callbackOrientation(self, msg):
        self.grAngle = msg.data

    def callbackPosX(self, msg):
        self.posX = msg.data

    def callbackPosY(self, msg):
        self.posY = msg.data

    def enableCheckDownCamLoop_cb(self, msg):
        self.enableCheckDownCamLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.enableCheckDownCamLoop:
            userdata.GRdist = [self.posX, self.posY]
            userdata.GRangle = self.grAngle
            if self.GRfound == False:
                self.enableTakeOffLoop_pub.publish(Bool(True))
                return 'CheckDownCam'
            else:
                self.enableTakeOffLoop_pub.publish(Bool(False))
                return 'FollowGR'
        else:
            return 'CheckDownCam'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")
    
class FollowGR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckDownCam'], input_keys=['GRdist', 'GRangle', 'minGoalAngle', 'maxGoalAngle'])
        self.pubPitchPID = rospy.Publisher('/IARC/DownCam/PitchPID', Float32, queue_size=1)
        self.pubRollPID = rospy.Publisher('/IARC/DownCam/RollPID', Float32, queue_size=1)

        self.enableStartInteractLoop_pub = rospy.Publisher('/IARC/states/enableStartInteractLoop', Bool, queue_size=1)

        rospy.Subscriber("/IARC/states/enableCheckDownCamLoop", Bool, callback=self.enableCheckDownCamLoop_cb)
        self.enableCheckDownCamLoop = True

    def enableCheckDownCamLoop_cb(self, msg):
        self.enableCheckDownCamLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        self.pubPitchPID.publish(Float32(userdata.GRdist[1]))
        self.pubRollPID.publish(Float32(userdata.GRdist[0]))
        if userdata.GRangle < userdata.minGoalAngle or userdata.GRangle > userdata.maxGoalAngle:
            self.enableStartInteractLoop_pub.publish(Bool(True))
        return 'CheckDownCam'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

############################## Interact Node ###############################
class StartInteract(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TouchDown', 'StartInteract'], input_keys=['lowHeight', 'altitudeDeviation', 'TouchDownTimerMAX'], output_keys=['TouchDownTimer'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        rospy.Subscriber("/IARC/currentAltitude", Float32, callback=self.callback)
        self.altitude = 0
        
        rospy.Subscriber("/IARC/states/enableStartInteractLoop", Bool, callback=self.enableStartInteractLoop_cb)
        self.enableStartInteractLoop = False

        self.enableCheckDownCamLoop_pub = rospy.Publisher('/IARC/states/enableCheckDownCamLoop', Bool, queue_size=1)

    def callback(self, msg):
        self.altitude = msg.data

    def enableStartInteractLoop_cb(self, msg):
        self.enableStartInteractLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.enableStartInteractLoop:
            self.pubTargetAltitude.publish(Float32(userdata.lowHeight))
            if abs(self.altitude - userdata.lowHeight) < userdata.altitudeDeviation:
                self.enableCheckDownCamLoop_pub.publish(Bool(False))
                userdata.TouchDownTimer = userdata.TouchDownTimerMAX + time.time()
                return 'TouchDown'
            else:
                return 'StartInteract'
        else:
            return 'StartInteract'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class TouchDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['AccendingCraft', 'TouchDown', 'StartInteract'], input_keys=['TouchDownTimer', 'groundHeight'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)

        rospy.Subscriber("/IARC/states/enableStartInteractLoop", Bool, callback=self.enableStartInteractLoop_cb)
        self.enableStartInteractLoop = False

    def enableStartInteractLoop_cb(self, msg):
        self.enableStartInteractLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        self.pubTargetAltitude.publish(Float32(userdata.groundHeight))
        if not self.enableStartInteractLoop:
            return 'StartInteract'
        if userdata.TouchDownTimer >= time.time():
            return 'AccendingCraft'
        else:
            return 'TouchDown'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class AccendingCraft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['AccendingCraft', 'StartInteract'], input_keys=['lowHeight', 'normHeight', 'altitudeDeviation'])
        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)
        rospy.Subscriber("/IARC/currentAltitude", Float32, callback=self.callback)
        self.altitude = 0

        self.enableCheckDownCamLoop_pub = rospy.Publisher('/IARC/states/enableCheckDownCamLoop', Bool, queue_size=1)
        self.enableStartInteractLoop_pub = rospy.Publisher('/IARC/states/enableStartInteractLoop', Bool, queue_size=1)

    def callback(self, msg):
        self.altitude = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        self.pubTargetAltitude.publish(Float32(userdata.normHeight))
        if self.altitude >= userdata.lowHeight:
            self.enableCheckDownCamLoop_pub.publish(Bool(True))
        if abs(self.altitude - userdata.normHeight) < userdata.altitudeDeviation:
            self.enableStartInteractLoop_pub.publish(Bool(False))
            return 'StartInteract'
        return 'AccendingCraft'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

############################# Obstacle Avoidence ############################
class CheckObstacles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckObstacles', 'ObstacleAvoidence'], input_keys=['obstacleThreshDist'])
        rospy.Subscriber("/IARC/Obstacle/dist", Float32, callback=self.callbackDist)
        self.dist = 2

    def callbackDist(self, msg):
        self.dist = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.dist <= userdata.obstacleThreshDist:
            return 'ObstacleAvoidence'
        else:
            return 'CheckObstacles'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class ObstacleAvoidence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckObstacles'], input_keys=['obstacleThreshDist'])
        rospy.Subscriber("/IARC/Obstacle/angle", Float32, callback=self.callbackAngle)
        rospy.Subscriber("/IARC/Obstacle/dist", Float32, callback=self.callbackDist)
        self.angle = 0
        self.dist = 2
        self.opposite_dist = 0
        self.pubPitchPID = rospy.Publisher('/IARC/Obstacle/PitchPID', Float32, queue_size=1)
        self.pubRollPID = rospy.Publisher('/IARC/Obstacle/RollPID', Float32, queue_size=1)

        self.enableTakeOffLoop_pub = rospy.Publisher('/IARC/states/enableTakeOffLoop', Bool, queue_size=1)
        self.enableCheckDownCamLoop_pub = rospy.Publisher('/IARC/states/enableCheckDownCamLoop', Bool, queue_size=1)
        self.enableStartInteractLoop_pub = rospy.Publisher('/IARC/states/enableStartInteractLoop', Bool, queue_size=1)

    def callbackDist(self, msg):
        self.dist = msg.data

    def callbackAngle(self, msg):
        self.angle = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        self.enableTakeOffLoop_pub.publish(Bool(True))
        self.enableCheckDownCamLoop_pub.publish(Bool(False))
        self.enableStartInteractLoop_pub.publish(Bool(False))

        self.opposite_dist = userdata.obstacleThreshDist - self.dist

        oppositeVec = (self.opposite_dist * math.cos(self.angle+math.pi), self.opposite_dist * math.sin(self.angle+math.pi))
        if DEBUG:
            print("Obstacle opposite vector:", oppositeVec)

        if self.opposite_dist > 0:
            self.pubRollPID.publish(oppositeVec[1])
            self.pubPitchPID.publish(oppositeVec[0])

        return 'CheckObstacles'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")
    
############################### Check Edges ##################################
class CheckEdges(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckEdges', 'EdgeTimer'], input_keys=['EdgeDetectTimerMAX'], output_keys=['EdgeDetectTimer'])
        rospy.Subscriber("/IARC/edgeDetect/detected", Bool, callback=self.callback)
        self.detected = False

    def callback(self, msg):
        self.detected = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.detected:
            userdata.EdgeDetectTimer = userdata.EdgeDetectTimerMAX + time.time()
            return 'EdgeTimer'
        else:
            return 'CheckEdges'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class EdgeTimer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckEdges', 'EdgeTimer', 'TowardsArena'], input_keys=['EdgeDetectTimer'])
        rospy.Subscriber("/IARC/edgeDetect/detected", Bool, callback=self.callback)
        self.detected = False

    def callback(self, msg):
        self.detected = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if not self.detected:
            return 'CheckEdges'
        if userdata.EdgeDetectTimer >= time.time():
            return 'TowardsArena'
        else:
            return 'EdgeTimer'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class TowardsArena(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckEdges', 'TowardsArena'], input_keys=[])
        rospy.Subscriber("/IARC/edgeDetect/detected", Bool, callback=self.callback)
        self.detected = False
        rospy.Subscriber("/IARC/edgeDetect/arenaVector/x", Float32, callback=self.arenaX_cb)
        rospy.Subscriber("/IARC/edgeDetect/arenaVector/y", Float32, callback=self.arenaY_cb)
        self.arenaX = 0
        self.arenaY = 0

        self.pubArenaX = rospy.Publisher('/IARC/edgeDetect/xPID', Float32, queue_size=1)
        self.pubArenaY = rospy.Publisher('/IARC/edgeDetect/yPID', Float32, queue_size=1)

    def callback(self, msg):
        self.detected = msg.data

    def arenaX_cb(self, msg):   
        self.arenaX = msg.data

    def arenaY_cb(self, msg):
        self.arenaY = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        self.pubArenaX.publish(Float32(self.arenaX))
        self.pubArenaY.publish(Float32(self.arenaY))
        if not self.detected:
            return 'CheckEdges'
        else:
            return 'TowardsArena'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

