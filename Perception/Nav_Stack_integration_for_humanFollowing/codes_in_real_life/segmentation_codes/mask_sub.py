#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize ROS node
rospy.init_node('mask_display_node')

# Initialize CvBridge to convert ROS image messages to OpenCV images
bridge = CvBridge()

# Callback function to process the received mask
def mask_callback(mask_msg):
    try:
        # Convert the ROS image message to OpenCV format
        mask_image = bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")

        # Display the mask image using OpenCV
        cv2.imshow("Received Mask", mask_image)
        cv2.waitKey(1)  # Display the frame

    except Exception as e:
        rospy.logerr(f"Error converting ROS message to OpenCV image: {e}")

# ROS subscriber for the mask topic
mask_sub = rospy.Subscriber("/segmentation/mask", Image, mask_callback)

# Keep the node running and listening to the mask topic
rospy.spin()

# Close all OpenCV windows on node shutdown
cv2.destroyAllWindows()
