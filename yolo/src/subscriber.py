#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, UInt8MultiArray
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2

class videosub():
    def __init__(self, topic):
        rospy.Subscriber(topic, UInt8MultiArray, self.callback)
        self.topic = topic
        self.image = np.zeros((480, 640, 3))

    def callback(self, data):
        self.image = cv2.imdecode(np.frombuffer(data.data, np.uint8), 1)

    def show(self):
        cv2.imshow('frame', self.image)
        cv2.waitKey(33)

    def getImage(self):
        return self.image

if __name__ == '__main__':
    rospy.init_node("Testnode")
    m = videosub("/sensor/forwardCam")
    # m.talker()
    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        m.show()
        rate.sleep()