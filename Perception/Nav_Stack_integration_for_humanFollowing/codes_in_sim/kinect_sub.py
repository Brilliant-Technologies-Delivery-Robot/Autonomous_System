#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, Bool  # Import Bool for the flag_human
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

# Configuration for the YOLOv8 model
MODEL_PATH = '/home/omar_ben_emad/delievery_robot_ws/src/yolov8/yolov8l.pt'

# List of needed objects to detect
# Needed = ["backpack", "bed", "bench", "bottle", "bowl", "cat", "chair", "cell phone", "coach", "cup", "dining table", "handbag", "person", "potted plant", "refrigerator", "vase", "tv"]
Needed = ['person']
frame_width = 640
frame_height = 480

class KinectObjectDetection:
    def __init__(self, threshold=0.65):
        self.bridge = CvBridge()
        
        # Load the YOLOv8 model
        self.model = YOLO(MODEL_PATH)
        
        self.threshold = threshold

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1
        self.font_thickness = 2
        self.color_static = (0, 255, 0)  # Green
        self.color_dynamic = (0, 0, 255)  # Red

        self.error_x_pub = rospy.Publisher('error_x', Int16, queue_size=10)
        self.error_y_pub = rospy.Publisher('error_y', Int16, queue_size=10)
        
        # Publisher for the bounding box center as a Point (x, y, z)
        self.bbox_center_pub = rospy.Publisher('bounding_box_center', Point, queue_size=10)
        rospy.init_node('kinect_object_detection', anonymous=True)
        # Subscribe to the Kinect RGB image topic
        rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)

        self.error_x = 0
        self.error_y = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (frame_width, frame_height))
        results = self.model(frame, conf=self.threshold, device='cuda')  # Specify device here
        
        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()  # Get bounding box coordinates
                class_index = box.cls[0].int().item()  # Get class index as integer

                # Check if class index is valid
                if class_index >= len(self.model.names):
                    rospy.logwarn(f"Class index {class_index} is out of bounds.")
                    continue
                
                label = self.model.names[class_index]  # Get class label
                confidence = box.conf[0].item()  # Get confidence score

                # Only process if the label is in the Needed list
                if confidence >= self.threshold and label in Needed:
                    rospy.loginfo(f"Detected: {label} with confidence {confidence:.2f}")
                    color = self.color_dynamic if label == "person" else self.color_static

                    current_center_x = (x1 + x2) / 2
                    current_center_y = (y1 + y2) / 2

                    # Calculate errors in x and y
                    self.error_x = current_center_x - frame_width // 2
                    self.error_y = current_center_y - frame_height // 2

                    # Draw bounding box and label
                    self.draw_simple_box(frame, (x1, y1, x2 - x1, y2 - y1), label, color)

                    # Check for person detection
                    if label == "person":
                        # Publish x, y, z position
                        center_point = Point(x=current_center_x, y=current_center_y, z=0)
                        self.bbox_center_pub.publish(center_point)  # Only publish center when a person is detected

                        # Print x, y, z position
                        rospy.loginfo(f"Person detected at: x={current_center_x}, y={current_center_y}")
                        # Draw the center of the bounding box for person
                        self.draw_center_with_opacity(frame, (int(current_center_x), int(current_center_y)))

                    # Draw the center of the bounding box
                    cv2.circle(frame, (int(current_center_x), int(current_center_y)), 5, (0, 255, 255), -1)

        self.draw_frame_center_lines(frame)
        self.display_frame(frame)

        # Publish the error_x and error_y after calculating them
        self.publish_error()  # Publish the errors after processing each frame

    def draw_simple_box(self, img, box, label, color):
        
        x, y, w, h = box
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        label_size, _ = cv2.getTextSize(label, self.font, self.font_scale, self.font_thickness)
        label_w, label_h = label_size
        cv2.rectangle(img, (x, y - label_h - 10), (x + label_w + 10, y), color, -1)
        cv2.putText(img, label, (x + 5, y - 5), self.font, self.font_scale, (0, 0, 0), self.font_thickness, cv2.LINE_AA)

    def draw_frame_center_lines(self, img):

        center_x = frame_width // 2
        center_y = frame_height // 2
        cv2.line(img, (0, center_y), (frame_width, center_y), (255, 255, 255), 1)
        cv2.line(img, (center_x, 0), (center_x, frame_height), (255, 255, 255), 1)

    def draw_center_with_opacity(self, img, center):

        overlay = img.copy()
        cv2.circle(overlay, center, 10, (0, 255, 255), -1)  # Yellow circle
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)

    def display_frame(self, frame):

        cv2.imshow('Detected Objects', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Close the frame if 'q' is pressed
            cv2.destroyAllWindows()  # Close the OpenCV window

    def publish_error(self):
        # Cast to integer before publishing
        self.error_x_pub.publish(int(self.error_x))
        self.error_y_pub.publish(int(self.error_y))

if __name__ == '__main__':
    try:
        KinectObjectDetection()  # Start object detection with Kinect camera
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
