#!/usr/bin/env python

import rospy
from subscriber import videosub
import os

import numpy as np
from keras.models import Model, load_model

class yolo():
    def __init__(self, model_file):
        self.model = load_model(model_file)
        self.grabCam = videosub("/sensor/forwardCam")

    def runYolo(self):
        image = self.grabCam.getImage()
        return self.model.predict(np.expand_dims(image, axis=0), batch_size=1)

    def postProcess(self, value):
        return value

if __name__ == '__main__':
    PATH = os.getcwd()
    rospy.init_node("yoloNode")
    yo = yolo(os.path.join(PATH, "the_file"))

    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        out = yo.runYolo()
        # insert post processing here

        rate.sleep()
