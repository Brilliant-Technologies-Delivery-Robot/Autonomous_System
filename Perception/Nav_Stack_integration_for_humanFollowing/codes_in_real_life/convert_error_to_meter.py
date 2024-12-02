#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Point

class ConvertToMeter:
    def __init__(self):
        # Initialize node
        rospy.init_node('convert_to_meter_node')
        
        # Subscribers to /error_x, /error_y, and /depth_center_point topics
        self.error_x_sub = rospy.Subscriber('/error_x', Int16, self.error_x_callback)
        self.error_y_sub = rospy.Subscriber('/error_y', Int16, self.error_y_callback)
        self.depth_sub = rospy.Subscriber('/depth_center_point', Point, self.depth_callback)

        # Publisher to /error_xy_meter topic (for Point message)
        self.error_xy_meter_pub = rospy.Publisher('/goal_to_nav_stack', Point, queue_size=10)

        # Initialize camera calibration parameters
        self.fx = 528.433756558705  # Focal length in pixels (x)
        self.fy = 528.433756558705  # Focal length in pixels (y)
        self.cx = 320.5  # Principal point (x) (center of image width)
        self.cy = 240.5  # Principal point (y) (center of image height)

        # Initialize variables to store error values and depth data
        self.error_x = 0
        self.error_y = 0
        self.depth_z = 0.0  # Z-value (depth) at the center of the object

    def error_x_callback(self, data):
        self.error_x = data.data
        self.convert_pixel_to_meter()

    def error_y_callback(self, data):
        self.error_y = data.data
        self.convert_pixel_to_meter()

    def depth_callback(self, data):
        # Retrieve the depth (Z) value from the Point message
        self.depth_z = data.z  # Z represents the depth at the center
        self.convert_pixel_to_meter()

    def convert_pixel_to_meter(self):
        if self.depth_z > 0:  # Ensure valid depth
            # Get pixel coordinates by adding error to principal point
            x_pixel = self.cx + self.error_x
            y_pixel = self.cy + self.error_y

            # Convert pixel errors to real-world coordinates (in meters)
            x_meter = (x_pixel - self.cx) * self.depth_z / self.fx
            y_meter = (y_pixel - self.cy) * self.depth_z / self.fy

            # Create a Point message and populate it with the converted values
            point_msg = Point()
            
            point_msg.x = self.depth_z
            point_msg.y = x_meter
            point_msg.z = 0

            # Publish the Point message to /error_xy_meter topic
            self.error_xy_meter_pub.publish(point_msg)

            rospy.loginfo(f"Converted to meters: X={x_meter}, Y={y_meter}, Z={self.depth_z}")
        else:
            rospy.logwarn("Invalid depth value")

if __name__ == '__main__':
    try:
        # Create instance of ConvertToMeter and keep the node running
        converter = ConvertToMeter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
