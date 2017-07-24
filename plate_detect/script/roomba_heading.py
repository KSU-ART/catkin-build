#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, UInt8MultiArray
import numpy as np
import cv2
from collections import deque
from math import atan2, sqrt, pi, sin, cos
import json
from subscriber import *

DEBUG = True


class FindAngle:
    def __init__(self):
        rospy.Subscriber("/yolo/second/boxes", String, callback=self.callback)
        
        self.AnglePub = rospy.Publisher("/IARC/OrientationNet/angle", Float32, queue_size=1)
        self.plateDetectXpub = rospy.Publisher("/IARC/OrientationNet/pos/x", Float32, queue_size=1)
        self.plateDetectYpub = rospy.Publisher("/IARC/OrientationNet/pos/y", Float32, queue_size=1)
        self.emptyYOLO = False
        self.predicted_angle = 0.0
        size = 15
        self.x_list = deque([0 for x in range(size)])
        self.y_list = deque([0 for x in range(size)])
        self.m = videosub("/sensor/compressed/downCam")

    # def show(self):
    #     cv2.imshow('frame', self.image)
    #     cv2.waitKey(33)

    def callback(self, msg):
        stringData = msg.data
        if DEBUG:
            print("YOLO string:", stringData)
        data = json.loads(stringData)
        # Check if YOLO message is empty (aka no ground robot detected)
        if len(data) == 0:
            self.emptyYOLO = True
            if DEBUG:
                print("YOLO is empty.")
        else:
            self.emptyYOLO = False
            # Store 
            coords = np.array(data)
            coords = coords[coords[:,0] < 2]

            # Subtract 0.5 from all of the boxes
            coords = coords - 0.5

            # Use distance formula on the x and y combinations to find smallest one, and convert floats x and y
            for i in range(coords.shape[0]):
                coords[i][0] = (sqrt(coords[i][1]*coords[i][1] + coords[i][2]*coords[i][2]))
                
            
            ind = np.argmin(coords, axis=0)

            # Pop the oldest value out and add the new one
            self.x_list.append(coords[ind[0]][1])
            self.y_list.append(coords[ind[0]][2])
            self.x_list.popleft()
            self.y_list.popleft()

            self.plateDetectXpub.publish(Float32(coords[ind[0]][1]))
            self.plateDetectYpub.publish(Float32(coords[ind[0]][2]))

            # Sum list
            x_sum = sum(self.x_list)
            y_sum = sum(self.y_list)

            # Take the arctangent to find the angle
            self.predicted_angle = atan2(y_sum, x_sum) #* 180 / pi

            if DEBUG:
                # print minargs
                print("Angle: ", self.predicted_angle)
            
            self.AnglePub.publish(Float32(self.predicted_angle))


            if DEBUG:
                cv_img = self.m.getProcessedImage()
                cv2.line(cv_img,(320,240),(int(100*cos(self.predicted_angle))+320, int(100*sin(self.predicted_angle))+240), (0,255,0), 5)
                # cv2.line(cv_img,(320,240),(int(100*pred[0][0])+320, int(-100*pred[0][1])+240), (255,0,0), 5)

                cv2.imshow('image',cv_img)
                cv2.waitKey(10)


if __name__ == '__main__':
    rospy.init_node("Testnode")
    findangle = FindAngle()
    rospy.spin()