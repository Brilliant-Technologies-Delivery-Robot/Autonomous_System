#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class SensorChecking:
    def __init__(self):
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Initialize the subscriber for the Kinect camera topic
        rospy.Subscriber('/rgb/image_raw', Image, self.camera_callback)
        
        # Initialize the publisher for the Kinect camera image
        self.kinect_img_pub = rospy.Publisher('/published/kinect/image_raw', Image, queue_size=10)
        
        # Placeholder variable for the image from the camera
        self.kinect_image = None

    def camera_callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            self.kinect_image = self.bridge.imgmsg_to_cv2(data)
            rospy.loginfo('Received image frame')
            self.pub_frames()
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def pub_frames(self):
        if self.kinect_image is not None:
            try:
                self.kinect_img_pub.publish(self.bridge.cv2_to_imgmsg(self.kinect_image))
                rospy.loginfo("Image Published with Shape: {0}".format(self.kinect_image.shape))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

    def show_images(self):
        # Display the image if it is received
        if self.kinect_image is not None:
            gray_image = cv2.cvtColor(self.kinect_image, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Kinect Camera Frame", gray_image)
            cv2.waitKey(10)

if __name__ == '__main__':
    rospy.init_node('Kinect_stream', anonymous=True)
    sensor_check = SensorChecking()
    
    # Main ROS loop
    rate = rospy.Rate(10)  # 10 Hz
    try:
        while not rospy.is_shutdown():
            sensor_check.show_images()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
