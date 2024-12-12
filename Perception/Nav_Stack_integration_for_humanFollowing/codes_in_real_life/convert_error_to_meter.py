#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16 , Float32
from geometry_msgs.msg import Point


class DepthIndicator:
    """
    Handles the depth indicator logic and state.
    """

    def __init__(self):
        self.value = 0

    def update(self, value):
        self.value = value
        rospy.loginfo(f"Depth Indicator Value: {self.value}")

    def is_invalid_depth(self):
        return self.value == 1


class CameraCalibration:
    """
    Stores camera calibration parameters for conversion.
    """

    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy


class PixelToMeterConverter:
    """
    Converts pixel coordinates and depth to real-world meters.
    """

    def __init__(self, calibration):
        self.calibration = calibration

    def convert(self, error_x, error_y, depth_z):
        """
        Converts pixel offsets and depth into real-world coordinates.
        """
        if depth_z <= 0:
            rospy.logwarn("Invalid depth value received for conversion")
            return None

        x_pixel = self.calibration.cx + error_x
        y_pixel = self.calibration.cy + error_y

        x_meter = (x_pixel - self.calibration.cx) * depth_z / self.calibration.fx
        y_meter = (y_pixel - self.calibration.cy) * depth_z / self.calibration.fy

        rospy.loginfo(f"Converted to meters: X={x_meter}, Y={y_meter}, Z={depth_z}")
        return Point(x=depth_z, y=x_meter, z=0)


class ErrorState:
    """
    Maintains the current error state (error_x, error_y) and depth (depth_z).
    """

    def __init__(self):
        self.error_x = 0
        self.error_y = 0
        self.depth_z = 0.0

    def update_error_x(self, value):
        self.error_x = value

    def update_error_y(self, value):
        self.error_y = value

    def update_depth(self, value):
        self.depth_z = value


class GoalPublisher:
    """
    Publishes the converted goal coordinates to a ROS topic.
    """

    def __init__(self, topic_name):
        self.publisher = rospy.Publisher(topic_name, Point, queue_size=10)

    def publish(self, point):
        self.publisher.publish(point)
        rospy.loginfo(f"Published Point: {point}")


class ConvertToMeterNode:
    """
    The main orchestrator that ties all components together.
    """

    def __init__(self):
        rospy.init_node('convert_to_meter_node')

        # Components adhering to SRP
        self.indicator = DepthIndicator()
        self.calibration = CameraCalibration(
            fx=528.433756558705,
            fy=528.433756558705,
            cx=320.5,
            cy=240.5
        )
        self.converter = PixelToMeterConverter(self.calibration)
        self.error_state = ErrorState()
        self.publisher = GoalPublisher('/goal_to_nav_stack')

        # Subscribers
        rospy.Subscriber('/error_x', Int16, self.error_x_callback)
        rospy.Subscriber('/error_y', Int16, self.error_y_callback)
        rospy.Subscriber('/depth_center_point', Float32, self.depth_callback)
        rospy.Subscriber('/depth_indicator', Int16, self.indicator_callback)

    def indicator_callback(self, data):
        """
        Updates the depth indicator value.
        """
        self.indicator.update(data.data)
        self.process_conversion()

    def error_x_callback(self, data):
        """
        Updates the error in the x-axis.
        """
        self.error_state.update_error_x(data.data)
        self.process_conversion()

    def error_y_callback(self, data):
        """
        Updates the error in the y-axis.
        """
        self.error_state.update_error_y(data.data)
        self.process_conversion()

    def depth_callback(self, data):
        """
        Updates the depth value from the Point message.
        """
        self.error_state.update_depth(data.data)
        self.process_conversion()

    def process_conversion(self):
        """
        Processes the conversion of pixel to real-world coordinates and publishes.
        """
        if self.indicator.is_invalid_depth():
            rospy.loginfo("Publishing zeros due to depth indicator being 1")
            self.publisher.publish(Point(0, 0, 0))
            return

        point = self.converter.convert(
            self.error_state.error_x,
            self.error_state.error_y,
            self.error_state.depth_z
        )

        if point:
            self.publisher.publish(point)


if __name__ == '__main__':
    try:
        # Create instance of ConvertToMeterNode and keep the node running
        node = ConvertToMeterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
