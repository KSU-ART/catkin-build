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

        self.stringData = "[]"

    def callback(self, msg):
        self.stringData = msg.data
        
    def enableTakeOffLoop_cb(self, msg):
        self.enableTakeOffLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
            print("yolo string:",self.stringData)
            print("emptyYOLO:", self.emptyYOLO)
        
        if not self.enableTakeOffLoop:
            return 'TakeOff'
		
        try:
        	self.enableCheckDownCamLoop_pub.publish(Bool(True))
        	data = json.loads(self.stringData)

        	# check if yolo message is empty (aka no ground robot detected)
        	if len(data) == 0:
        	    self.emptyYOLO = True
        	    if DEBUG:
        	        print("yolo is empty")
        	else:
        	    self.emptyYOLO = False
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
	
	        if self.emptyYOLO:
	            # no ground robot detected
	            print("going to Random Traversal")
	            return 'RandomTraversal'
	        else:
	            if self.minYolo != None:
	                userdata.targetYolo = self.minYolo
	            return 'FindGR'
        except:
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
        
        self.YawPIDrt.publish(Float32(self.angle - self.targetAngle))
        return 'FindGR'
    
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
        smach.State.__init__(self, outcomes=['CheckDownCam'], input_keys=['GRdist', 'GRangle', 'maxGoalAngle'])
        self.pubPitchPID = rospy.Publisher('/IARC/DownCam/PitchPID', Int16, queue_size=1)
        self.pubRollPID = rospy.Publisher('/IARC/DownCam/RollPID', Int16, queue_size=1)
        rospy.Subscriber("/IARC/currentAngle", Float32, callback=self.callback)
        self.angle = 0
        self.targetAngle = None
        self.angleK = 135

        self.enableStartInteractLoop_pub = rospy.Publisher('/IARC/states/enableStartInteractLoop', Bool, queue_size=1)

        self.StartInteract_pub = rospy.Publisher('/IARC/states/StartInteract', Bool, queue_size=1)

        rospy.Subscriber("/IARC/states/enableCheckDownCamLoop", Bool, callback=self.enableCheckDownCamLoop_cb)
        self.enableCheckDownCamLoop = False

    def normalizeAngle(self, x):
        if(x > math.pi):
            x -= 2*math.pi
        elif(x < -math.pi):
            x += 2*math.pi
        return x

    def enableCheckDownCamLoop_cb(self, msg):
        self.enableCheckDownCamLoop = msg.data

    def callback(self, msg):
        self.angle = msg.data
        if(self.targetAngle == None):
            self.targetAngle = self.normalizeAngle(self.angle + (self.angleK * math.pi /180))

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
            print("userdata.GRangle", userdata.GRangle)
            print("userdata.maxGoalAngle", userdata.maxGoalAngle)
            print("Actual Angle", userdata.GRangle+self.angle)
            print("userdata.GRdist", userdata.GRdist)
        if self.enableCheckDownCamLoop:
            self.pubPitchPID.publish(Int16(userdata.GRdist[1]))
            self.pubRollPID.publish(Int16(userdata.GRdist[0]))
            # find the angle of ground robot relative to ground
            # then check to see if it is within target angle
            if abs( self.normalizeAngle( self.normalizeAngle(userdata.GRangle + self.angle) - self.targetAngle) ) > userdata.maxGoalAngle:
                self.StartInteract_pub.publish(Bool(True))
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
            print("userdata.TouchDownTimer: ", userdata.TouchDownTimer)
            print("time.time(): ", time.time())
            print("delta: ", userdata.TouchDownTimer - time.time())
        self.pubTargetAltitude.publish(Float32(userdata.groundHeight))
        if not self.enableStartInteractLoop:
            return 'StartInteract'
        if userdata.TouchDownTimer < time.time():
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

        self.EndInteract_pub = rospy.Publisher('/IARC/states/EndInteract', Bool, queue_size=1)

        self.enableCheckDownCamLoop_pub = rospy.Publisher('/IARC/states/enableCheckDownCamLoop', Bool, queue_size=1)
        self.enableStartInteractLoop_pub = rospy.Publisher('/IARC/states/enableStartInteractLoop', Bool, queue_size=1)

        rospy.Subscriber("/IARC/states/enableStartInteractLoop", Bool, callback=self.enableStartInteractLoop_cb)
        self.enableStartInteractLoop = False

    def enableStartInteractLoop_cb(self, msg):
        self.enableStartInteractLoop = msg.data

    def callback(self, msg):
        self.altitude = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if not self.enableStartInteractLoop:
            return 'StartInteract'

        self.pubTargetAltitude.publish(Float32(userdata.normHeight))
        if self.altitude >= userdata.lowHeight:
            self.enableCheckDownCamLoop_pub.publish(Bool(True))
        if abs(self.altitude - userdata.normHeight) < userdata.altitudeDeviation:
            self.enableStartInteractLoop_pub.publish(Bool(False))
            self.EndInteract_pub.publish(Bool(True))
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

        rospy.Subscriber("/IARC/states/enableTakeOffLoop", Bool, callback=self.enableTakeOffLoop_cb)
        rospy.Subscriber("/IARC/states/enableCheckDownCamLoop", Bool, callback=self.enableCheckDownCamLoop_cb)
        rospy.Subscriber("/IARC/states/enableStartInteractLoop", Bool, callback=self.enableStartInteractLoop_cb)
        rospy.Subscriber("/IARC/states/enableEdgeLoop", Bool, callback=self.enableEdgeLoop_cb)
        self.enableTakeOffLoop = True
        self.enableCheckDownCamLoop = False
        self.enableStartInteractLoop = False
        self.enableEdgeLoop = True

        self.enableTakeOffLoop_pub = rospy.Publisher('/IARC/states/enableTakeOffLoop', Bool, queue_size=1)

        self.enableObstacleLoop_pub = rospy.Publisher('/IARC/states/enableObstacleLoop', Bool, queue_size=1)

    def callbackDist(self, msg):
        self.dist = msg.data

    def enableTakeOffLoop_cb(self, msg):
        self.enableTakeOffLoop = msg.data

    def enableCheckDownCamLoop_cb(self, msg):
        self.enableCheckDownCamLoop = msg.data

    def enableStartInteractLoop_cb(self, msg):
        self.enableStartInteractLoop = msg.data

    def enableEdgeLoop_cb(self, msg):
        self.enableEdgeLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.dist <= userdata.obstacleThreshDist:
            self.enableObstacleLoop_pub.publish(Bool(True))
            return 'ObstacleAvoidence'
        else:
            self.enableObstacleLoop_pub.publish(Bool(False))
            if self.enableEdgeLoop == False:
                if self.enableTakeOffLoop == False and self.enableCheckDownCamLoop == False and self.enableStartInteractLoop == False:
                    self.enableTakeOffLoop_pub.publish(Bool(True))
            return 'CheckObstacles'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class ObstacleAvoidence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckObstacles'], input_keys=['normHeight', 'obstacleThreshDist'])
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

        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)

    def callbackDist(self, msg):
        self.dist = msg.data

    def callbackAngle(self, msg):
        self.angle = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        print("\n*******Obstacle avoidence*******\n")
        self.pubTargetAltitude.publish(Float32(userdata.normHeight))
        
        self.enableTakeOffLoop_pub.publish(Bool(False))
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

        rospy.Subscriber("/IARC/states/enableTakeOffLoop", Bool, callback=self.enableTakeOffLoop_cb)
        rospy.Subscriber("/IARC/states/enableCheckDownCamLoop", Bool, callback=self.enableCheckDownCamLoop_cb)
        rospy.Subscriber("/IARC/states/enableStartInteractLoop", Bool, callback=self.enableStartInteractLoop_cb)
        rospy.Subscriber("/IARC/states/enableObstacleLoop", Bool, callback=self.enableObstacleLoop_cb)
        self.enableTakeOffLoop = True
        self.enableCheckDownCamLoop = False
        self.enableStartInteractLoop = False
        self.enableObstacleLoop = True

        self.enableTakeOffLoop_pub = rospy.Publisher('/IARC/states/enableTakeOffLoop', Bool, queue_size=1)

        self.enableEdgeLoop_pub = rospy.Publisher('/IARC/states/enableEdgeLoop', Bool, queue_size=1)

    def callback(self, msg):
        self.detected = msg.data

    def enableTakeOffLoop_cb(self, msg):
        self.enableTakeOffLoop = msg.data

    def enableCheckDownCamLoop_cb(self, msg):
        self.enableCheckDownCamLoop = msg.data

    def enableStartInteractLoop_cb(self, msg):
        self.enableStartInteractLoop = msg.data

    def enableObstacleLoop_cb(self, msg):
        self.enableObstacleLoop = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if self.enableObstacleLoop == False:
            if self.enableTakeOffLoop == False and self.enableCheckDownCamLoop == False and self.enableStartInteractLoop == False:
                self.enableTakeOffLoop_pub.publish(Bool(True))
        
        if self.detected:
            self.enableEdgeLoop_pub.publish(Bool(True))
            userdata.EdgeDetectTimer = userdata.EdgeDetectTimerMAX + time.time()
            if DEBUG:
                print("****************Starting Edge Detect******************")
            return 'EdgeTimer'
        else:
            self.enableEdgeLoop_pub.publish(Bool(False))
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

        self.enableTakeOffLoop_pub = rospy.Publisher('/IARC/states/enableTakeOffLoop', Bool, queue_size=1)
        self.enableCheckDownCamLoop_pub = rospy.Publisher('/IARC/states/enableCheckDownCamLoop', Bool, queue_size=1)
        self.enableStartInteractLoop_pub = rospy.Publisher('/IARC/states/enableStartInteractLoop', Bool, queue_size=1)

    def callback(self, msg):
        self.detected = msg.data

    def execute(self, userdata):
        if DEBUG:
            time.sleep(1)
        if not self.detected:
            return 'CheckEdges'
        if userdata.EdgeDetectTimer < time.time():
            if DEBUG:
                print("****************Edge Detect TIMER Engaged******************")
            self.enableTakeOffLoop_pub.publish(Bool(False))
            self.enableCheckDownCamLoop_pub.publish(Bool(False))
            self.enableStartInteractLoop_pub.publish(Bool(False))
            return 'TowardsArena'
        else:
            return 'EdgeTimer'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

class TowardsArena(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CheckEdges', 'TowardsArena'], input_keys=['normHeight'])
        rospy.Subscriber("/IARC/edgeDetect/detected", Bool, callback=self.callback)
        self.detected = False
        rospy.Subscriber("/IARC/edgeDetect/arenaVector/x", Float32, callback=self.arenaX_cb)
        rospy.Subscriber("/IARC/edgeDetect/arenaVector/y", Float32, callback=self.arenaY_cb)
        self.arenaX = 0
        self.arenaY = 0

        self.pubArenaX = rospy.Publisher('/IARC/edgeDetect/xPID', Float32, queue_size=1)
        self.pubArenaY = rospy.Publisher('/IARC/edgeDetect/yPID', Float32, queue_size=1)

        self.pubTargetAltitude = rospy.Publisher('/IARC/setAltitude', Float32, queue_size=1)

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
        self.pubTargetAltitude.publish(Float32(userdata.normHeight))
        if not self.detected:
            return 'CheckEdges'
        else:
            return 'TowardsArena'
    
    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")

