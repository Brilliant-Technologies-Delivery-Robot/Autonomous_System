#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from ultralytics import YOLO
import numpy as np

class sub_detect_face():
    def __init__(self):
        # Initialize the CvBridge object
        self.bridge = CvBridge()
        self.facemodel = YOLO('/home/omar_ben_emad/delievery_robot_ws/src/face_detect/yolov8n-face.pt')
        self.image_sub = rospy.Subscriber("/camera_frame", Image, self.image_callback)
        self.error_x_pub = rospy.Publisher('/face_real_error_x', Int16, queue_size=10)
        self.error_x = 0
        
        # Set up a timer to publish the error at a regular rate (10 Hz)
        self.publish_rate = 10  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_error)

        self.frame_height = 640
        self.frame_width = 480
        self.detection_conf = 0.4

        self.opacity = 0.4  # Adjust opacity here (0.0 to 1.0)
        self.color_one = (0, 255, 0)
        self.color_two = (255, 0, 0)
        self.color_three = (0, 0, 255)
        self.color_four = (255, 255, 255)

        self.center_line_thickness = 2
        self.bounding_box_thickness = 2
        self.center_circle_radius = 10
        self.face_center_radius = 2
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to a CV2 Image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Resize the frame
            frame = cv2.resize(cv_image, (self.frame_height, self.frame_width))
            height, width, _ = frame.shape
            # Calculate the center coordinates of the frame
            frame_center_x = width // 2
            frame_center_y = height // 2

            overlay = frame.copy()
            # Draw horizontal and vertical center lines on the overlay
            cv2.line(overlay, (0, frame_center_y), (width, frame_center_y), self.color_one, self.center_line_thickness)  # Horizontal line
            cv2.line(overlay, (frame_center_x, 0), (frame_center_x, height), self.color_one, self.center_line_thickness)  # Vertical line
            # Draw a circle at the center of the frame on the overlay
            cv2.circle(overlay, (frame_center_x, frame_center_y), self.center_circle_radius, self.color_two, -1)  # Blue circle

            face_result = self.facemodel(frame)[0]  # YOLOv8 returns a list, we need the first result
            largest_area = 0  # Reset largest area for each frame
            largest_bbox = None  # Reset largest bbox for each frame

            for box in face_result.boxes:
                if box.conf >= self.detection_conf:  # Ensure the detection confidence is above the threshold
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    area = w * h

                    # Update the largest bounding box if the current area is larger
                    if area > largest_area:
                        largest_area = area
                        largest_bbox = (x1, y1, x2, y2)

            # Draw only the largest bounding box
            if largest_bbox:
                x1, y1, x2, y2 = largest_bbox
                w, h = x2 - x1, y2 - y1
                
                # Draw rectangle around the largest face with a thicker border
                cv2.rectangle(frame, (x1, y1), (x2, y2), self.color_one, self.bounding_box_thickness)
                
                # Calculate the center of the largest face
                face_center_x = (x1 + x2) // 2
                face_center_y = (y1 + y2) // 2
                
                # Draw a circle at the center of the face
                cv2.circle(frame, (face_center_x, face_center_y), self.face_center_radius, self.color_three, -1)  # Red circle
                
                # Calculate the error between the center of the face and the center of the frame
                self.error_x = face_center_x - frame_center_x
                error_y = face_center_y - frame_center_y

                # Draw text annotations
                cv2.putText(frame, f'Face Center: ({face_center_x}, {face_center_y})', 
                            (x1, y1 - 10), self.font, 0.5, self.color_four, 2)
                cv2.putText(frame, f'Error X: {self.error_x}', 
                            (10, height - 10), self.font, 0.5, self.color_four, 2)

                # Print the detection box coordinates, centers, and error
                rospy.loginfo(f"Largest Face detected: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                rospy.loginfo(f"Center of the face: ({face_center_x}, {face_center_y})")
                rospy.loginfo(f"Error in X : ({self.error_x})")
                rospy.loginfo(f"Error in Y : ({error_y})")
            else:
                # Print the center of the frame if no face is detected
                rospy.loginfo(f"No face detected. Center of the frame: ({frame_center_x}, {frame_center_y})")

            # Blend the overlay with the original frame
            cv2.addWeighted(overlay, self.opacity, frame, 1 - self.opacity, 0, frame)
            cv2.imshow('Face Detection Frame', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                rospy.signal_shutdown('User requested shutdown')

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def publish_error(self, event):
        # Publish the current error value
        self.error_x_pub.publish(Int16(data=self.error_x))

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('face_detect_yolo', anonymous=True)       
        yolo_camera_processor = sub_detect_face()
        yolo_camera_processor.spin()
    except rospy.ROSInterruptException:
        pass
