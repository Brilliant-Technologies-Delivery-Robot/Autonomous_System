#!/usr/bin/env python3

import cv2 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from std_msgs.msg import String, Int8  # Import Int8 for publishing integers

class ObjectDetectorNode:
    def __init__(self):
        self.sub_node = 'camera_subscriber'
        self.topic_name = '/published/kinect/image_raw'

        # Define file paths
        self.config_file = '/home/omar_ben_emad/new_delivery/src/sdd_detection/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        self.frozen_model = '/home/omar_ben_emad/new_delivery/src/sdd_detection/frozen_inference_graph.pb'
        self.file_name = '/home/omar_ben_emad/new_delivery/src/sdd_detection/labels.txt'
        self.model = cv2.dnn_DetectionModel(self.frozen_model, self.config_file)

        self.classLabels = self.load_labels()

        self.model.setInputSize(320, 320)
        self.model.setInputScale(1.0 / 127.5)
        self.model.setInputMean((127.5, 127.5, 127.5))
        self.model.setInputSwapRB(True)

        self.font_scale = 2
        self.font = cv2.FONT_HERSHEY_PLAIN

        rospy.init_node(self.sub_node, anonymous=True)
        rospy.Subscriber(self.topic_name, Image, self.callback)
        self.label_publisher = rospy.Publisher("/detected_object_label", String, queue_size=10)
        self.center_pub = rospy.Publisher('bounding_box_center', Point, queue_size=10)
        self.detection_flag_pub = rospy.Publisher("/detection_flag", Int8, queue_size=10)  # New publisher for detection flag

        rospy.spin()

    def load_labels(self):
        try:
            with open(self.file_name, 'rt') as fpt:
                return fpt.read().rstrip('\n').split('\n')
        except Exception as e:
            rospy.logfatal(f"Failed to load labels: {e}")
            raise RuntimeError(f"Failed to load labels: {e}")

    def callback(self, msg):
        bridgeObj = CvBridge()
        rospy.loginfo("Received Successfully")
        conv_frame = bridgeObj.imgmsg_to_cv2(msg)
        conv_frame = cv2.cvtColor(conv_frame, cv2.COLOR_BGR2RGB)
        classIndex, confidence, bbox = self.model.detect(conv_frame, confThreshold=0.65)

        # Get frame dimensions
        height, width, _ = conv_frame.shape

        # Create an overlay with the same dimensions as the frame
        overlay = conv_frame.copy()

        # Calculate the center coordinates of the frame
        frame_center_x = width // 2
        frame_center_y = height // 2

        # Draw horizontal and vertical center lines on the overlay
        cv2.line(overlay, (0, frame_center_y), (width, frame_center_y), (0, 255, 0), 2)  # Horizontal line
        cv2.line(overlay, (frame_center_x, 0), (frame_center_x, height), (0, 255, 0), 2)  # Vertical line

        # Draw the center point of the frame on the overlay
        center_point_overlay = overlay.copy()
        cv2.circle(center_point_overlay, (frame_center_x, frame_center_y), 10, (255, 0, 0), -1)  # Red circle

        # Blend the center point overlay with the original frame
        cv2.addWeighted(center_point_overlay, 0.5, overlay, 0.5, 0, overlay)

        # Blend the center lines overlay with the original frame
        alpha = 0.5  # 50% opacity for center lines
        cv2.addWeighted(overlay, alpha, conv_frame, 1 - alpha, 0, conv_frame)

        detected = False

        if len(classIndex) != 0:
            detected = True
            for classInd, boxes in zip(classIndex.flatten(), bbox):
                if classInd <= 80:
                    label = self.classLabels[classInd - 1]
                    self.label_publisher.publish(label)
                    rospy.loginfo(label)
                    cv2.rectangle(conv_frame, boxes, (255, 0, 0), 2)
                    cv2.putText(conv_frame, label, (boxes[0] + 10, boxes[1] + 40), self.font, fontScale=1, color=(0, 255, 0), thickness=2)
                    
                    # Calculate center point of current bounding box
                    current_center_x = boxes[0] + boxes[2] / 2
                    current_center_y = boxes[1] + boxes[3] / 2
                    
                    rospy.loginfo(f"Bounding box coordinates: {boxes}")
                    rospy.loginfo(f"Center coordinates: x = {current_center_x}, y = {current_center_y}")

                    self.publish_center_point(current_center_x, current_center_y)
                    cv2.circle(conv_frame, (int(current_center_x), int(current_center_y)), 5, (0, 255, 255), -1)

        # Publish detection flag based on whether an object was detected
        detection_flag = 1 if detected else 0
        self.detection_flag_pub.publish(detection_flag)
        
        # Resize frame for display
        frame_resized = cv2.resize(conv_frame, (640, 480))
        cv2.imshow('frame', frame_resized)
        cv2.waitKey(1)

    def publish_center_point(self, x, y):
        center_point = Point(x, y, 0)
        self.center_pub.publish(center_point)

if __name__ == '__main__':
    ObjectDetectorNode()
