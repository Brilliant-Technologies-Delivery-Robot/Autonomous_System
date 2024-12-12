#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Point


class DepthImageConverter:
    def __init__(self):
        self.bridge = CvBridge()

    def convert(self, ros_image):
        try:
            # Convert ROS Image to OpenCV format (depth image)
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            if np.max(depth_image) > 10:
                depth_image = depth_image / 1000.0  # Convert from mm to meters if necessary
            return depth_image
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return None


class DepthErrorCalculator:
    def __init__(self, desired_x, desired_y, min_depth, max_depth):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.min_depth = min_depth
        self.max_depth = max_depth

    def calculate_errors(self, center_x, center_y):
        return self.desired_x - center_x, self.desired_y - center_y

    def is_valid_depth(self, depth_value):
        return self.min_depth <= depth_value <= self.max_depth


class DepthImageVisualizer:
    def display_image(self, depth_image, center_x=None, center_y=None, desired_x=None, desired_y=None):
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        if center_x is not None and center_y is not None:
            cv2.circle(depth_image_normalized, (int(center_x), int(center_y)), 5, (0, 255, 255), -1)
        if desired_x is not None and desired_y is not None:
            cv2.circle(depth_image_normalized, (desired_x, desired_y), 5, (255, 255, 0), -1)
        cv2.imshow("Depth Image", depth_image_normalized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested shutdown.")
            cv2.destroyAllWindows()


class DepthPublisher:
    def __init__(self):
        self.error_x_publisher = rospy.Publisher('error_x', Int16, queue_size=10)
        self.error_y_publisher = rospy.Publisher('error_y', Int16, queue_size=10)
        self.depth_publisher = rospy.Publisher('/depth_center_point', Float32, queue_size=10)

    def publish_errors(self, error_x, error_y):
        self.error_x_publisher.publish(Int16(error_x))
        self.error_y_publisher.publish(Int16(error_y))

    def publish_depth(self, depth_value):
        self.depth_publisher.publish(Float32(depth_value))


class DepthImageProcessor:
    def __init__(self):
        rospy.init_node('depth_image_listener', anonymous=True)

        self.converter = DepthImageConverter()
        self.error_calculator = DepthErrorCalculator(desired_x=320, desired_y=240, min_depth=0.25, max_depth=10.0)
        self.visualizer = DepthImageVisualizer()
        self.publisher = DepthPublisher()

        self.point_one = None
        self.point_two = None

        rospy.Subscriber('/camera/depth_registered/image', Image, self.image_callback)
        rospy.Subscriber('bounding_box_start', Point, self.bounding_box_start_callback)
        rospy.Subscriber('bounding_box_end', Point, self.bounding_box_end_callback)

    def bounding_box_start_callback(self, data):
        self.point_one = (int(data.x), int(data.y))
        rospy.loginfo(f"Received bounding box start: {self.point_one}")

    def bounding_box_end_callback(self, data):
        self.point_two = (int(data.x), int(data.y))
        rospy.loginfo(f"Received bounding box end: {self.point_two}")

    def apply_median_filter(self, depth_image, kernel_size=5):
        # Apply median filter using OpenCV (CPU)
        depth_image_filtered = cv2.medianBlur(depth_image, kernel_size)
        return depth_image_filtered

    def image_callback(self, msg):
        depth_image = self.converter.convert(msg)
        if depth_image is None:
            return

        if self.point_one and self.point_two:
            x1, y1 = self.point_one
            x2, y2 = self.point_two

            # Calculate center
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            # Create a writable copy of the depth image for drawing
            depth_image_copy = depth_image.copy()

            # Draw bounding box and calculate errors
            cv2.rectangle(depth_image_copy, self.point_one, self.point_two, (0, 255, 0), 2)

            if self.is_within_bounds(center_x, center_y, depth_image):
                # Apply median filter on the depth image
                filtered_depth_image = self.apply_median_filter(depth_image_copy, kernel_size=5)  # Use 5x5 kernel by default

                # Get the filtered depth value at the center
                depth_value = filtered_depth_image[int(center_y), int(center_x)]

                if self.error_calculator.is_valid_depth(depth_value):
                    self.handle_valid_depth(center_x, center_y, depth_value)
                else:
                    rospy.logwarn(f"Filtered depth at ({center_x}, {center_y}) out of range: {depth_value}")

            else:
                rospy.logwarn(f"Center coordinates ({center_x}, {center_y}) are out of bounds")

            self.visualizer.display_image(
                filtered_depth_image, 
                center_x, center_y, 
                self.error_calculator.desired_x, self.error_calculator.desired_y
            )

    def handle_valid_depth(self, center_x, center_y, depth_value):
        error_x, error_y = self.error_calculator.calculate_errors(center_x, center_y)
        rospy.loginfo(f"Error X: {error_x}, Error Y: {error_y}, Depth: {depth_value}")

        # Ensure error_x and error_y are integers before publishing
        self.publisher.publish_errors(int(error_x), int(error_y))  # Explicitly cast to int
        self.publisher.publish_depth(depth_value)
        
    def is_within_bounds(self, x, y, image):
        return 0 <= x < image.shape[1] and 0 <= y < image.shape[0]

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    processor = DepthImageProcessor()
    processor.run()

"""
To stabilize and  get steady dedpth values i have used the following approaches:

1. Exponential Moving Average (EMA) to the depth values in the bounding box area, you'll need to modify the code so that the depth values within the bounding box are smoothed over time.
But i have found out that it was very costy in the computation power so i didn't used it

2. The same case happend with the SMA but it was even worse in the results not 

3. For now the m ost optimum option was the Median filter 

"""

'''
It would be more accurate to apply the median filter over mask of detected object
'''