#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

class CameraSubPub:
    def __init__(self):
        # Initialize the CvBridge object
        self.bridge = CvBridge()

        # Initialize the ROS node
        rospy.init_node('camera_sub_pub_node', anonymous=True)

        # Subscribe to the image and camera info topics
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        # Publisher for the processed image and camera info
        self.image_pub = rospy.Publisher("/mono_camera/image_processed", Image, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to a CV2 Image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Display the image
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)

            # Convert the CV2 image back to a ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            # Publish the processed image
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def spin(self):
        rospy.spin()
        # Destroy the OpenCV window on shutdown
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        camera_sub_pub = CameraSubPub()
        camera_sub_pub.spin()
    except rospy.ROSInterruptException:
        pass
