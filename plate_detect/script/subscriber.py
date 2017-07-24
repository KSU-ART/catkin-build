#!/usr/bin/env python
# simple cv buffer subscriber object, created by https://github.com/stoplime

import rospy
from std_msgs.msg import UInt8, UInt8MultiArray
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2

class videosub():
    def __init__(self, topic, img_data_shape=(416, 416), display_shape=(640, 480)):
        rospy.Subscriber(topic, UInt8MultiArray, self.callback)
        self.shape = img_data_shape
        self.display = display_shape
        self.topic = topic
        self.image = None
        self.newImgAvailable = False
        print('Subscriber object created, listening to', topic)

    def callback(self, data):
        self.image = cv2.imdecode(np.frombuffer(data.data, np.uint8), 1)
        self.newImgAvailable = True

    def show(self):
        cv2.imshow('frame', self.image)
        cv2.waitKey(33)

    def getProcessedImage(self):#note: swapaxes for the net, image stays in regular format
        self.newImgAvailable = False
        return self.image

if __name__ == '__main__':
    rospy.init_node("Testnode")
    m = videosub("/sensor/forwardCam")

    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        m.show()
        rate.sleep()