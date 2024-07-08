#!/usr/bin/env python3
import cv2 
import math
import os
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String  # Import String message type for publishing strings
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSensorPublisher:
    def __init__(self):
        self.pub_node_name = 'camera_sensor_pub'
        self.topic_name = 'obj_detect'
        rospy.init_node(self.pub_node_name, anonymous=True)
        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=100)
        self.cap = cv2.VideoCapture(0)
        self.bridgeObj = CvBridge()

    def publish_camera_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret == True:
                rospy.loginfo("Published Successfully")
                img = self.bridgeObj.cv2_to_imgmsg(frame)
                self.publisher.publish(img)

if __name__ == "__main__":
    camera_sensor_publisher = CameraSensorPublisher()
    camera_sensor_publisher.publish_camera_image()
