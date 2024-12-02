#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import Point

class DepthImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.center_x = None
        self.center_y = None
        self.point_one_x = None
        self.point_one_y = None

        rospy.init_node('depth_image_listener', anonymous=True)
        rospy.Subscriber("/camera/depth_registered/image", Image, self.image_callback)
        rospy.Subscriber('bounding_box_center', Point, self.point_callback)
        rospy.Subscriber('point_one_topic', Point, self.point_one_callback)  # Subscriber for second point

        # Publishers for individual errors
        self.error_x_publisher = rospy.Publisher('error_x', Int16, queue_size=10)
        self.error_y_publisher = rospy.Publisher('error_y', Int16, queue_size=10)

        # Publisher for the depth as z in Point message
        self.depth_point_publisher = rospy.Publisher('depth_center_point', Point, queue_size=10)

        # Desired Points (these represent the center of the image)
        self.desired_x = 320
        self.desired_y = 240

        # Depth clipping parameters
        self.min_depth = 0.25  # Minimum depth in meters
        self.max_depth = 10.0  # Maximum depth in meters

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert depth values to meters if they are in millimeters
            if np.max(depth_image) > 10:  # Assuming depth values in millimeters
                depth_image = depth_image / 1000.0

            # Handle NaN values
            depth_image = np.nan_to_num(depth_image, nan=0.0)

            if self.center_x is not None and self.center_y is not None:
                # Convert center coordinates to integer
                center_x = int(self.center_x)
                center_y = int(self.center_y)

                # Ensure center coordinates are within image bounds
                if 0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]:
                    # Get the depth value at the center point
                    depth_at_center = depth_image[center_y, center_x]

                    # Ensure depth value is valid and within the desired range
                    if np.isfinite(depth_at_center) and self.min_depth <= depth_at_center <= self.max_depth:
                        # Calculate the errors
                        error_x = self.desired_x - center_x
                        error_y = self.desired_y - center_y

                        # Print the errors and depth
                        rospy.loginfo(f"Error X: {error_x}, Error Y: {error_y}")
                        rospy.loginfo(f"Depth at Center: {depth_at_center}")

                        # Publish the individual errors
                        self.publish_errors(error_x, error_y)

                        # Publish the depth as z in Point message
                        point_msg = Point(x=center_x, y=center_y, z=depth_at_center)
                        self.depth_point_publisher.publish(point_msg)
                    else:
                        rospy.logwarn(f"Depth value at center {depth_at_center} is out of range or invalid")
                else:
                    rospy.logwarn("Center point is out of bounds")

            # Optionally, display the depth image with the center point
            self.show_depth_image(depth_image)

        except CvBridgeError as e:
            rospy.logerr(e)

    def point_callback(self, data):
        rospy.loginfo("Received center point: x = %f, y = %f", data.x, data.y)
        self.center_x = data.x
        self.center_y = data.y

    def point_one_callback(self, data):
        rospy.loginfo("Received point one: x = %f, y = %f", data.x, data.y)
        self.point_one_x = data.x
        self.point_one_y = data.y

    def publish_errors(self, error_x, error_y):
        # Publish the individual error messages
        self.error_x_publisher.publish(Int16(error_x))
        self.error_y_publisher.publish(Int16(error_y))

    def show_depth_image(self, depth_image):
        # Normalize the depth image for display
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_normalized = np.uint8(depth_image_normalized)

        # Optionally draw the center point
        if self.center_x is not None and self.center_y is not None:
            cv2.circle(depth_image_normalized, (int(self.center_x), int(self.center_y)), 5, (0, 255, 255), -1)
        # Display the image

        cv2.circle(depth_image_normalized, (self.desired_x, self.desired_y), 5, (255, 255, 0), -1)
        cv2.imshow("Depth Image with Center Points", depth_image_normalized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User requested shutdown.")

        # Optionally print some statistics for debugging
        rospy.loginfo(f"Max Depth: {np.max(depth_image)} meters")
        rospy.loginfo(f"Min Depth: {np.min(depth_image)} meters")
        rospy.loginfo(f"Mean Depth: {np.mean(depth_image)} meters")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    depth_processor = DepthImageProcessor()
    depth_processor.run()
