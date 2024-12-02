#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStreamPublisher:
    def __init__(self):
        # Initialize the ROS node
        self.image_pub = rospy.Publisher('/camera_frame', Image, queue_size=10)
        self.bridge = CvBridge()
        # Open the camera (0 is usually the default laptop camera)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera!")
            return
        self.rate = rospy.Rate(10)

    def stream_camera(self):
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logerr("Failed to capture image!")
                break

            # Display the frame in a window
            cv2.imshow("Camera Feed", frame)
            
            # Convert the frame to a ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(image_msg)
            
            # Check for a keyboard interrupt (press 'q' to exit)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            self.rate.sleep()

        # Release the camera and close all OpenCV windows
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('camera_stream_publisher', anonymous=True)       
        camera_publisher = CameraStreamPublisher()
        camera_publisher.stream_camera()
    except rospy.ROSInterruptException:
        pass
