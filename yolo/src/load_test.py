'''

'''
import os
import tensorflow as tf
import numpy as np
from keras.models import load_model

from yad2k.models.keras_yolo import yolo_body
from yad2k.models.keras_yolo import yolo_eval, yolo_head


# prediction as int32multiarray with len() 6*(# of boxes)
# int32multiarray format: [class_num_0, out_box_0, out_score_0, ..., class_num_n, out_box_n, out_score_n]
# OR
# json string list([classNum, centerX, centerY])

class yolo(object):
    def __init__(self, score_threshold=0.3, iou_threshold=0.6):
        # Set up paths.
        filepath = os.path.dirname(os.path.abspath(__file__))

        model_path = os.path.join(filepath, 'model_data', 'trained_body.h5')
        anchors_path = os.path.join(filepath, 'model_data', 'yolo_anchors.txt')
        classes_path = os.path.join(filepath, 'model_data', 'aerial_classes.txt')

        # Load classes and anchors.
        with open(classes_path) as f:
                class_names = f.readlines()
        class_names = [c.strip() for c in class_names]

        with open(anchors_path) as f:
            anchors = f.readline()
            anchors = [float(x) for x in anchors.split(',')]
            anchors = np.array(anchors).reshape(-1, 2)

        # Load model and set up computation graph.
        self.sess = K.get_session()

        self.yolo_body = load_model(model_path)

        self.yolo_body.summary()

        self.yolo_outputs = yolo_head(self.yolo_body.output, anchors, len(class_names))

        self.input_image_shape = K.placeholder(shape=(2, ))

        self.boxes, self.scores, self.classes = yolo_eval(
            self.yolo_outputs,
            self.input_image_shape,
            score_threshold=score_threshold,
            iou_threshold=iou_threshold)

    # Make predictions for one or more images, in (batch, height, width, channel) format
    # assert len(image_s.shape) == 4 # image(s) ha(s/ve) 4 dims ready to be sent into the graph
    def pred(image_s):
        out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_body.input: image_s,
                    self.input_image_shape: [image_s.shape[1], image_s.shape[2]],
                    K.learning_phase(): 0
                })

        return (out_boxes, out_scores, out_classes)
