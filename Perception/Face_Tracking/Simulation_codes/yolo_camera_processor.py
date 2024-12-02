#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from ultralytics import YOLO
import numpy as np

class YoloCameraProcessor:
    def __init__(self):
        # Initialize the CvBridge object
        self.bridge = CvBridge()
        self.facemodel = YOLO('/home/omar_ben_emad/delievery_robot_ws/src/face_detect/yolov8n-face.pt')

        rospy.init_node('yolo_camera_processor', anonymous=True)
        self.image_sub = rospy.Subscriber("/mono_camera/image_processed", Image, self.image_callback)
        self.error_x_pub = rospy.Publisher('/face_error_x', Int16, queue_size=100)
        self.error_x = 0
        self.publish_rate = 100  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_error)

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to a CV2 Image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Resize the frame
            frame = cv2.resize(cv_image, (640, 480))

            # Get frame dimensions
            height, width, _ = frame.shape

            # Calculate the center coordinates of the frame
            frame_center_x = width // 2
            frame_center_y = height // 2

            # Create an overlay with the same dimensions as the frame
            overlay = frame.copy()

            # Draw horizontal and vertical center lines on the overlay
            cv2.line(overlay, (0, frame_center_y), (width, frame_center_y), (0, 255, 0), 2)  # Horizontal line
            cv2.line(overlay, (frame_center_x, 0), (frame_center_x, height), (0, 255, 0), 2)  # Vertical line

            # Draw a circle at the center of the frame on the overlay
            center_circle_radius = 10
            cv2.circle(overlay, (frame_center_x, frame_center_y), center_circle_radius, (255, 0, 0), -1)  # Blue circle

            # Perform face detection
            face_result = self.facemodel(frame, conf=0.3)

            # Initialize variables to track the largest bounding box
            largest_area = 0
            largest_bbox = None

            for info in face_result:
                parameters = info.boxes
                for box in parameters:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    h, w = y2 - y1, x2 - x1
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
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Calculate the center of the largest face
                face_center_x = (x1 + x2) // 2
                face_center_y = (y1 + y2) // 2
                
                # Draw a circle at the center of the face
                cv2.circle(frame, (face_center_x, face_center_y), 2, (0, 0, 255), -1)  # Red circle
                
                # Calculate the error between the center of the face and the center of the frame
                self.error_x = face_center_x - frame_center_x
                error_y = face_center_y - frame_center_y

                # Draw text annotations
                cv2.putText(frame, f'Face Center: ({face_center_x}, {face_center_y})', 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f'Error X: {self.error_x}', 
                            (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Print the detection box coordinates, centers, and error
                rospy.loginfo(f"Largest Face detected: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                rospy.loginfo(f"Center of the face: ({face_center_x}, {face_center_y})")
                rospy.loginfo(f"Error in X : ({self.error_x})")
                rospy.loginfo(f"Error in Y : ({error_y})")
            else:
                # Print the center of the frame if no face is detected
                rospy.loginfo(f"No face detected. Center of the frame: ({frame_center_x}, {frame_center_y})")

            # Print the center of the frame every frame
            rospy.loginfo(f"Center of the frame: ({frame_center_x}, {frame_center_y})")

            # Blend the overlay with the original frame
            opacity = 0.4  # Adjust opacity here (0.0 to 1.0)
            cv2.addWeighted(overlay, opacity, frame, 1 - opacity, 0, frame)

            # Display the frame with a title
            cv2.imshow('Face Detection Frame', frame)
            
            # Exit if 'q' is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.signal_shutdown('User requested shutdown')

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def publish_error(self, event):
        # Publish the current error value at a higher rate
        self.error_x_pub.publish(Int16(data=self.error_x))

    def spin(self):
        rospy.spin()
        # Destroy the OpenCV window on shutdown
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        yolo_camera_processor = YoloCameraProcessor()
        yolo_camera_processor.spin()
    except rospy.ROSInterruptException:
        pass
