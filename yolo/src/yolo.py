#!/usr/bin/env python
from __future__ import print_function

import os
import rospy
import cv2
import tensorflow as tf
import numpy as np
from keras.models import load_model
from keras import backend as K

from yad2k.utils.draw_boxes import draw_boxes
from yad2k.models.keras_yolo import yolo_body, yolo_eval, yolo_head
from subscriber import videosub

filepath = os.path.dirname(os.path.abspath(__file__))

class yolo(object):
    '''
    YOLOv2 class integrated with YAD2K and ROS
    '''
    def __init__(self, score_threshold=0.3, iou_threshold=0.6, max_detections=15):
        # Set up paths.
        model_path = os.path.join(filepath, 'model_data', 'trained_body.h5')
        anchors_path = os.path.join(filepath, 'model_data', 'yolo_anchors.txt')
        classes_path = os.path.join(filepath, 'model_data', 'aerial_classes.txt')

        # Load classes and anchors.
        with open(classes_path) as f:
                self.class_names = f.readlines()
        self.class_names = [c.strip() for c in self.class_names]

        with open(anchors_path) as f:
            self.anchors = f.readline()
            self.anchors = [float(x) for x in self.anchors.split(',')]
            self.anchors = np.array(self.anchors).reshape(-1, 2)

        # Load model and set up computation graph.
        self.sess = K.get_session()

        self.yolo_body = load_model(model_path)

        self.yolo_body.summary()

        self.yolo_outputs = yolo_head(self.yolo_body.output, self.anchors, len(self.class_names))

        self.input_image_shape = K.placeholder(shape=(2, ))

        self.boxes, self.scores, self.classes = yolo_eval(
            self.yolo_outputs,
            self.input_image_shape,
            max_boxes=max_detections,
            score_threshold=score_threshold,
            iou_threshold=iou_threshold)

        print('yolo object created')

    def pred(self, image_s):
        # Make predictions for one or more images, in (batch, height, width, channel) format
        assert len(image_s.shape) == 4 # image must have 4 dims ready to be sent into the graph
        out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_body.input: image_s,
                    self.input_image_shape: [image_s.shape[1], image_s.shape[2]],
                    K.learning_phase(): 0
                })
        return out_boxes, out_scores, out_classes

    def display(self, out_boxes, out_scores, out_classes, image, name):
        if len(out_boxes) == 0:
            cv2.imshow(name, np.floor(image[0] * 255 + 0.5).astype('uint8'))
        else:
            image = draw_boxes(image[0], out_boxes, out_classes, self.class_names, scores=out_scores)
            cv2.imshow(name, image)
        cv2.waitKey(10)

if __name__ == '__main__':
    PATH = os.getcwd()
    rospy.init_node("yoloNode")

    yo = yolo(.2,.3, 100)

    vid1 = videosub("/sensor/forwardCam")
    vid2 = videosub("/sensor/forwardCam")

    rate = rospy.Rate(5)
    image = np.ndarray([1] + list(vid1.image.shape))
    while not rospy.is_shutdown():
        # Grab new images from the subscribers
        if vid1.newImgAvailable:
            image = vid1.getProcessedImage()
            boxes, scores, classes = yo.pred(image)
            yo.display(boxes, scores, classes, image, vid1.topic) # Display
        if vid2.newImgAvailable:
            image = vid2.getProcessedImage()
            boxes, scores, classes = yo.pred(image)
            yo.display(boxes, scores, classes, image, vid2.topic)

        rate.sleep()
