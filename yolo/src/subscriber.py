#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_msgs.msg import UInt8, UInt8MultiArray
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2

class videosub():
    def __init__(self, topic):
        rospy.Subscriber(topic, UInt8MultiArray, self.callback)
        self.topic = topic
        self.image = np.ndarray((480, 640, 3))
        self.newImgAvailable = False
        print('Subscriber object created, listening to', topic)

    def callback(self, data):
        self.image = cv2.imdecode(np.frombuffer(data.data, np.uint8), 1)
        self.newImgAvailable = True

    def show(self):
        cv2.imshow('frame', self.image)
        cv2.waitKey(33)

    def getImage(self):
        self.newImgAvailable = False
        return np.expand_dims(np.array(cv2.resize(self.image, (416, 416)), dtype='float32')*0.003921568, axis=0)

if __name__ == '__main__':
    rospy.init_node("Testnode")
    m = videosub("/sensor/forwardCam")

    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        m.show()
        rate.sleep()