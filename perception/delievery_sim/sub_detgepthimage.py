#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point

class DepthImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        self.fx = 525.0  # focal length in pixels
        self.fy = 525.0
        self.cx = 320.0  # principal point in pixels
        self.cy = 240.0

        # Define the bounding box coordinates (xmin, ymin, xmax, ymax)
        self.bbox_xmin = 0
        self.bbox_ymin = 0
        self.bbox_xmax = 0
        self.bbox_ymax = 0
        
        self.point_x1 = 0
        self.point_x2 = 0
        self.point_y1 = 0
        self.point_y2 = 0

        rospy.init_node('depth_image_listener', anonymous=True)
        rospy.Subscriber("/depth/image_raw", Image, self.image_callback)
        rospy.Subscriber('point_one_topic', Point, self.point_one_callback)
        rospy.Subscriber('point_two_topic', Point, self.point_two_callback)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Update the bounding box coordinates based on points received
            bbox_xmin = self.point_x1
            bbox_ymin = self.point_y1
            bbox_xmax = self.point_x2 + self.point_x1
            bbox_ymax = self.point_y2 + self.point_y1

            bbox_xmin = int(bbox_xmin)
            bbox_ymin = int(bbox_ymin)
            bbox_xmax = int(bbox_xmax)
            bbox_ymax = int(bbox_ymax)

            # Extract the region of interest (ROI) within the bounding box
            roi_depth_image = depth_image[bbox_ymin:bbox_ymax, bbox_xmin:bbox_xmax]

            # Filter out NaN values within the ROI
            self.valid_depths = roi_depth_image[np.isfinite(roi_depth_image)]

            # Print all valid depth values and the number of valid points within the bounding box
            if self.valid_depths.size > 0:
                print(f"Depth values within bounding box: {self.valid_depths}")
                print(f"Number of valid depth points: {self.valid_depths.size}")
            else:
                print("No valid depth values found within bounding box.")

            # Make a copy of the depth image for displaying the bounding box
            depth_image_copy = depth_image.copy()

            # Optionally, display the depth image with the bounding box
            cv2.rectangle(depth_image_copy, (bbox_xmin, bbox_ymin), (bbox_xmax, bbox_ymax), (255, 0, 0), 2)
            cv2.imshow("Depth Image with Bounding Box", depth_image_copy)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def point_one_callback(self, data):
        rospy.loginfo("Received point one: x = %f, y = %f, z = %f", data.x, data.y, data.z)
        self.point_x1 = data.x
        self.point_y1 = data.y

    def point_two_callback(self, data):
        rospy.loginfo("Received point two: x = %f, y = %f, z = %f", data.x, data.y, data.z)
        self.point_x2 = data.x
        self.point_y2 = data.y

    def get_pointcloud(self):
        for i in range(self.valid_depths.size):
            pass
        pass
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    depth_processor = DepthImageProcessor()
    depth_processor.run()
