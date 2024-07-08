#!/usr/bin/env python3

import cv2 
import math
import os
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sub_node = 'camera_subscriber'
topic_name= 'obj_detect'

config_file = '/home/omar/delieveryRobotPerception/src/sdd/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
frozen_model = '/home/omar/delieveryRobotPerception/src/sdd/frozen_inference_graph.pb'

model = cv2.dnn_DetectionModel(frozen_model, config_file)

classLabels = []
file_name = '/home/omar/delieveryRobotPerception/src/sdd/labels.txt'
with open(file_name,'rt') as fpt:
    classLabels = fpt.read().rstrip('\n').split('\n')


model.setInputSize(320,320)
model.setInputScale(1.0/127.5)
model.setInputMean((127.5,127.5,127.5))
model.setInputSwapRB(True)

font_scale = 2
font = cv2.FONT_HERSHEY_PLAIN

def callback(msg):
    bridgeObj = CvBridge()
    rospy.loginfo("Received Successfuly")
    conv_frame =bridgeObj.imgmsg_to_cv2(msg)
    classIndex, confidence, bbox = model.detect(conv_frame , confThreshold=0.65)  #tune the confidence  as required
    conv_frame = cv2.UMat(conv_frame)

    if (len(classIndex) != 0 ):
        for classInd, boxes in zip(classIndex.flatten(), bbox):
            if (classInd<=80):
                print(boxes)
                rospy.loginfo(classLabels[classInd-1])

                cv2.rectangle(conv_frame, boxes, (255, 0, 0), 2)
                cv2.putText(conv_frame, classLabels[classInd-1], (boxes[0] + 10, boxes[1] + 40), font, fontScale = 1, color=(0, 255, 0), thickness=2)
                       
    cv2.imshow('frame',conv_frame) 
    cv2.waitKey(1)
    

rospy.init_node(sub_node,anonymous=True)
rospy.Subscriber(topic_name , Image , callback)
rospy.spin()
cv2.destroyAllWindows()


