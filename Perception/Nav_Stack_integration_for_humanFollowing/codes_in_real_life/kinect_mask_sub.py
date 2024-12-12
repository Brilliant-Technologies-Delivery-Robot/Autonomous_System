#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

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


class DepthImageVisualizer:
    def display_images(self, depth_image, bitwise_image=None):
        # Normalize the depth image for display
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Create a placeholder for the final display
        if bitwise_image is not None:
            # Normalize the bitwise image
            bitwise_image_normalized = cv2.normalize(bitwise_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            # Combine depth map and bitwise output side by side
            combined_image = np.hstack((depth_image_normalized, bitwise_image_normalized))
        else:
            combined_image = depth_image_normalized

        # Display the combined image
        cv2.imshow("Depth Map (Left) | Bitwise Output (Right)", combined_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested shutdown.")
            cv2.destroyAllWindows()

class DepthImageProcessor:
    def __init__(self):
        rospy.init_node('depth_image_listener', anonymous=True)

        self.converter = DepthImageConverter()
        self.visualizer = DepthImageVisualizer()
        self.mask_image = None  # To store the latest mask image

        # Publisher for the median depth
        self.goal_publisher = rospy.Publisher('/depth_center_point', Float32, queue_size=10)

        # Subscribe to both the mask and depth image topics
        rospy.Subscriber("/segmentation/mask", Image, self.mask_callback)
        rospy.Subscriber('/camera/depth_registered/image', Image, self.image_callback)

    # Callback function to process the received mask
    def mask_callback(self, data):
        bridge = CvBridge()
        try:
            # Convert the ROS image message to OpenCV format
            mask_image = bridge.imgmsg_to_cv2(data, desired_encoding="mono8")
            # Store the mask image
            self.mask_image = mask_image

        except Exception as e:
            rospy.logerr(f"Error converting ROS message to OpenCV image: {e}")

    def image_callback(self, msg):
        depth_image = self.converter.convert(msg)
        if depth_image is None:
            return

        # Remove invalid depth values (NaN and Inf)
        depth_image_cleaned = depth_image[np.isfinite(depth_image)]

        # Check if there are valid depth values left
        if depth_image_cleaned.size > 0:
            # Calculate min and max depth values
            min_depth = np.min(depth_image_cleaned)
            max_depth = np.max(depth_image_cleaned)
            rospy.loginfo(f"Min Depth: {min_depth}, Max Depth: {max_depth}")
        else:
            rospy.logwarn("No valid depth values in the image.")

        bitwise_output = None  # Default if no mask is provided
        if self.mask_image is not None:
            # Resize the mask to match the depth image
            mask_resized = cv2.resize(self.mask_image, (depth_image.shape[1], depth_image.shape[0]))

            # Perform bitwise AND using the mask
            mask_depth_values = np.float32(depth_image) * (mask_resized.astype(np.float32) > 0).astype(np.float32)

            # Calculate the median depth value
            valid_depths = mask_depth_values[mask_depth_values > 0]  # Consider only valid depths (non-zero)
            if valid_depths.size > 0:
                median_depth = np.median(valid_depths)
                rospy.loginfo(f"Median Depth over Mask: {median_depth} meters")
                
                # Publish the median depth to the topic
                self.goal_publisher.publish(median_depth)
            else:
                rospy.logwarn("No valid depth values over the mask region.")

            # Store the bitwise output for visualization
            bitwise_output = mask_depth_values

        # Pass both images to the visualizer
        self.visualizer.display_images(depth_image, bitwise_output)
            
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    processor = DepthImageProcessor()
    processor.run()
