#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Int16, Bool  # Import Bool for the flag_human

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Point

class DepthImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.center_x = None
        self.center_y = None

        rospy.init_node('depth_image_listener', anonymous=True)
        rospy.Subscriber("/depth/image_raw", Image, self.image_callback)
        rospy.Subscriber('bounding_box_center', Point, self.point_callback)
        # Publisher for errors and depth as Point message
        self.error_point_publisher = rospy.Publisher('error_and_depth', Point, queue_size=10)

        # Desired Points
        self.desired_x = 320
        self.desired_y = 240

        # Depth clipping parameters
        self.min_depth = 0.5  # Minimum depth in meters
        self.max_depth = 10.0  # Maximum depth in meters

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if self.center_x is not None and self.center_y is not None:
                # Convert center coordinates to integer
                center_x = int(self.center_x)
                center_y = int(self.center_y)

                # Ensure center coordinates are within image bounds
                if 0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]:
                    # Get the depth value at the center point
                    depth_at_center = depth_image[center_y, center_x]
                    if np.isfinite(depth_at_center) and self.min_depth <= depth_at_center <= self.max_depth:
                        # Calculate the errors
                        self.error_x = center_x - self.desired_x 
                        self.error_y = center_y - self.desired_y
                        self.publish_point(self.error_x, self.error_y, depth_at_center)
                    else:
                        rospy.logwarn(f"Depth value at center {depth_at_center} is out of range or invalid")
                else:
                    rospy.logwarn("Center point is out of bounds")

        except CvBridgeError as e:
            rospy.logerr(e)

    def point_callback(self, data):
        rospy.loginfo("Received center point: x = %f, y = %f", data.x, data.y)
        self.center_x = data.x
        self.center_y = data.y

    def publish_point(self, error_x, error_y, depth_at_center):
 
        error_point = Point(x=error_x, y=error_y, z=depth_at_center)
        self.error_point_publisher.publish(error_point)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    depth_processor = DepthImageProcessor()
    depth_processor.run()
