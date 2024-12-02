#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Bool  # Import Bool for the flag_human
from geometry_msgs.msg import Point

class ConvertToMeter:
    def __init__(self):
        # Initialize node
        rospy.init_node('convert_to_meter_node')
        
        # Subscribers to /error_and_depth and /flag_human topics
        self.error_xy_sub = rospy.Subscriber('/error_and_depth', Point, self.error_xy_callback)
        self.error_xy_meter_pub = rospy.Publisher('/error_xy_meter', Point, queue_size=10)

        self.fx = 528.433756558705  # Focal length in pixels (x)
        self.fy = 528.433756558705  # Focal length in pixels (y)
        self.cx = 320.5  # Principal point (x) (center of image width)
        self.cy = 240.5  # Principal point (y) (center of image height)
        # Initialize variables to store error values and depth data
        self.error_x = 0
        self.error_y = 0
        self.depth_z = 0.0  # Z-value (depth) at the center of the object

    def error_xy_callback(self, data):
        # Assuming the Point message contains x and y errors as well as depth z
        self.error_x = data.x  # Get the pixel error in x
        self.error_y = data.y  # Get the pixel error in y
        self.depth_z = data.z   # Get the depth (Z) value
        self.convert_pixel_to_meter()

    def convert_pixel_to_meter(self):
        if self.depth_z > 0.5:  # Ensure valid depth
            # Get pixel coordinates by adding error to principal point
            x_pixel = self.cx + self.error_x
            y_pixel = self.cy + self.error_y

            # Convert pixel errors to real-world coordinates (in meters)
            x_meter = (x_pixel - self.cx) * self.depth_z / self.fx
            y_meter = (y_pixel - self.cy) * self.depth_z / self.fy

            # Create a Point message and populate it with the converted values
            point_msg = Point()
            point_msg.x = x_meter
            point_msg.y = y_meter
            point_msg.z = self.depth_z  # Include the Z value (depth) in the Point message

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
