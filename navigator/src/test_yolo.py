#!/usr/bin/env python

import json
import rospy

from std_msgs.msg import String

def main():
    rospy.init_node('Test Yolo')
    yoloPub = rospy.Publisher("/IARC/YOLO", String, queue_size=1)

    yolo_value = [[0,320,240]]
    yolo_json = json.dumps(yolo_value)
    while not rospy.is_shutdown():
        yoloPub.publish(String(yolo_json))
        rospy.spin()
    

if __name__ == '__main__':
    main()