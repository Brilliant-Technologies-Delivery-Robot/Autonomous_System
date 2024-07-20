#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Define the bounding box coordinates (xmin, ymin, xmax, ymax)
bbox_xmin = 304
bbox_ymin = 9
bbox_xmax = 455
bbox_ymax = 315

def callback(data):
    # Convert PointCloud2 data to a list of points
    point_list = []
    for point in pc2.read_points(data, skip_nans=True):
        point_list.append(point)

    roi_depth_image = point_list[bbox_ymin:bbox_ymax, bbox_xmin:bbox_xmax]
    print(roi_depth_image)
    valid_pcl = roi_depth_image[np.isfinite(roi_depth_image)]

    # Do something with the point_list
    print("Received {} points".format(len(point_list)))

def listener():
    # Initialize the ROS node
    rospy.init_node('point_cloud_listener', anonymous=True)
    
    # Subscribe to the /depth/points topic
    rospy.Subscriber("/depth/points", PointCloud2, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
