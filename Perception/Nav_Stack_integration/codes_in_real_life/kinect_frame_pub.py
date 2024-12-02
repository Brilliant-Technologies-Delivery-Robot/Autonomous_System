#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class KinectPublisher:
    def __init__(self):
        rospy.init_node('kinect_publisher', anonymous=True)
        
        # Publisher to publish the RGB frames to a new topic
        self.rgb_pub = rospy.Publisher('/camera/rgb/published_image', Image, queue_size=10)
        
        # Subscriber to get the RGB frames from the Kinect
        rospy.Subscriber("/camera/rgb/image_color", Image, self.rgb_callback)
        
        self.bridge = CvBridge()
        
        rospy.spin()

    def rgb_callback(self, data):
        rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Published RGB Image", rgb_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User requested shutdown.")
        
        # Convert OpenCV image back to ROS Image message and publish it
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        self.rgb_pub.publish(rgb_msg)

if __name__ == '__main__':
    try:
        KinectPublisher()
    except rospy.ROSInterruptException:
        pass
