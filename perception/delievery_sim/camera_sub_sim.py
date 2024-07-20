#!/usr/bin/env python3

import cv2 
import math
import os
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from std_msgs.msg import String

class ObjectDetectorNode:
    def __init__(self):

        self.sub_node = 'camera_subscriber'
        self.topic_name= '/published/kinect/image_raw'
        self.points_img_topic = '/depth/points'

        self.config_file = '/home/omar/delieveryRobotPerception/src/sdd/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        self.frozen_model = '/home/omar/delieveryRobotPerception/src/sdd/frozen_inference_graph.pb'
        self.model = cv2.dnn_DetectionModel(self.frozen_model, self.config_file)
        self.classLabels = []
        self.file_name = '/home/omar/delieveryRobotPerception/src/sdd/labels.txt'
        self.load_labels()

        self.model.setInputSize(320,320)
        self.model.setInputScale(1.0/127.5)
        self.model.setInputMean((127.5,127.5,127.5))
        self.model.setInputSwapRB(True)

        self.font_scale = 2
        self.font = cv2.FONT_HERSHEY_PLAIN

        rospy.init_node(self.sub_node, anonymous=True)
        rospy.Subscriber(self.topic_name, Image, self.callback)
        self.label_publisher = rospy.Publisher("/detected_object_label", String, queue_size=10)
        self.pointOne_pub = rospy.Publisher('point_one_topic', Point, queue_size=10)
        self.pointTwo_pub = rospy.Publisher('point_two_topic', Point, queue_size=10)

        self.detected_objects_count = 0
        rospy.spin()

    def load_labels(self):
        with open(self.file_name,'rt') as fpt:
            self.classLabels = fpt.read().rstrip('\n').split('\n')

    def callback(self, msg):
        bridgeObj = CvBridge()
        rospy.loginfo("Received Successfully")
        conv_frame = bridgeObj.imgmsg_to_cv2(msg)
        conv_frame = cv2.cvtColor(conv_frame, cv2.COLOR_BGR2RGB)
        classIndex, confidence, bbox = self.model.detect(conv_frame, confThreshold=0.65)

        if len(classIndex) != 0:
            for classInd, boxes in zip(classIndex.flatten(), bbox):
                if classInd <= 80:
                    label = self.classLabels[classInd - 1]
                    self.label_publisher.publish(label)
                    rospy.loginfo(label)
                    cv2.rectangle(conv_frame, boxes, (255, 0, 0), 2)
                    cv2.putText(conv_frame, label, (boxes[0] + 10, boxes[1] + 40), self.font, fontScale=1, color=(0, 255, 0), thickness=2)
                    
                    # Calculate center point of current bounding box
                    current_center_x = (boxes[0]) + boxes[2] / 2
                    current_center_y = (boxes[1]) + boxes[3] / 2
                    
                    rospy.loginfo(boxes[0])
                    rospy.loginfo(boxes[1])
                    rospy.loginfo(boxes[2])
                    rospy.loginfo(boxes[3])

                    self.publish_points(boxes[0], boxes[1], boxes[2], boxes[3])
                    cv2.circle(conv_frame, (int(current_center_x), int(current_center_y)), 5, (0, 255, 255), -1)       
                    
        cv2.imshow('frame', conv_frame)
        cv2.waitKey(1)

    def publish_points(self, x1, y1, x2, y2):
        point_one = Point(x1, y1, 0)
        point_two = Point(x2, y2, 0)
        self.pointOne_pub.publish(point_one)
        self.pointTwo_pub.publish(point_two)

if __name__ == '__main__':
    ObjectDetectorNode()
