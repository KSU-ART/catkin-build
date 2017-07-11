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
    def __init__(self, score_threshold=0.3, iou_threshold=0.6):
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
            score_threshold=score_threshold,
            iou_threshold=iou_threshold)

        print('yolo object created')

    def pred(self, image_s):
        # Make predictions for one or more images, in (batch, height, width, channel) format
        assert len(image_s.shape) == 4 # image(s) ha(s/ve) 4 dims ready to be sent into the graph
        out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_body.input: image_s,
                    self.input_image_shape: [image_s.shape[1], image_s.shape[2]],
                    K.learning_phase(): 0
                })
        return (out_boxes, out_scores, out_classes)

    def display(self, out_boxes, out_scores, out_classes, image, mode):
        if mode == 0:
            forward_image = draw_boxes(image[0], out_boxes, out_classes, self.class_names, scores=out_scores)
            cv2.imshow('forwardCam', forward_image)
        if  mode == 2: # 2 images
            forward_image = draw_boxes(image[0], out_boxes[0], out_classes[0], self.class_names, scores=out_scores[0])
            cv2.imshow('forwardCam', forward_image)
            down_image = draw_boxes(image[1], out_boxes[1], out_classes[1], self.class_names, scores=out_scores[1])
            cv2.imshow('downCam', down_image)
        else: # mode = 1
            down_image = draw_boxes(image[0], out_boxes, out_classes, self.class_names, scores=out_scores)
            cv2.imshow('downCam', down_image)
        cv2.waitKey(33)

if __name__ == '__main__':
    PATH = os.getcwd()
    rospy.init_node("yoloNode")

    yo = yolo()

    vid1 = videosub("/sensor/forwardCam")
    vid2 = videosub("/sensor/ddownCam")

    image = None
    mode = 0
    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        # Grab new images from the subscribers
        if vid1.newImgAvailable:
            image = vid1.getImage()
            mode = 0 # forwardcamCam
            if vid2.newImgAvailable:
                image = np.concatenate(image, vid2.getImage(), axis=0)
                mode = 2 # forwardCam and downCam
        elif vid2.newImgAvailable:
            image = vid2.getImage()
            mode = 1 # downCam
        
        # Get model predictions
        if not (image is None):
            boxes, scores, classes = yo.pred(image)
            print('found {} boxes'.format(len(boxes)))
            print(boxes, classes)
            if len(boxes) != 0:
                yo.display(boxes, scores, classes, image, mode)
            image = None

        rate.sleep()
