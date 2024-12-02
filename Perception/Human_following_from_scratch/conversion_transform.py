#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped

# class ObjectTransformer:
#     def __init__(self):
#         rospy.init_node('object_transformer', anonymous=True)
        
#         # Initialize the TF2 buffer and listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#         # Subscriber to the /error_xy_meter topic (x, y, z points)
#         self.error_xy_meter_sub = rospy.Subscriber('/error_xy_meter', Point, self.xy_callback)
        
#         # Publisher to the /transformed_cord topic (transformed x, y, z points)
#         self.transformed_cord_pub = rospy.Publisher('/transformed_cord', Point, queue_size=10)

#     def xy_callback(self, data):
#         # Extract x, y, z from the Point message
#         detected_point_camera = [data.x, data.y, data.z]
#         # Perform the transformation to base_link
#         transformed_point = self.transform_point(detected_point_camera)
#         # Publish the transformed point
#         if transformed_point:
#             self.publish_transformed_point(transformed_point)

#     def transform_point(self, point_in_camera_frame):
#         # Create a PointStamped message for the detected point in the camera_optical_frame
#         point_stamped = PointStamped()
#         point_stamped.header.frame_id = "camera_frame_optical"  # Change this to your actual camera optical frame ID
#         point_stamped.header.stamp = rospy.Time.now()
#         point_stamped.point.x = point_in_camera_frame[0]  # Depth (z)
#         point_stamped.point.y = point_in_camera_frame[1]  # Horizontal offset (x)
#         point_stamped.point.z = point_in_camera_frame[2]  # Vertical offset (y)

#         try:
#             # Lookup the transformation from camera_optical_frame to base_link
#             transform = self.tf_buffer.lookup_transform("base_link", "camera_frame_optical", rospy.Time(0), rospy.Duration(1.0))
            
#             # Transform the point into the base_link frame
#             point_in_base_link = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
#             rospy.loginfo(f"Transformed Point in base_link: x={point_in_base_link.point.x}, y={point_in_base_link.point.y}, z={point_in_base_link.point.z}")
#             return point_in_base_link.point
        
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             rospy.logerr(f"Transformation failed: {e}")
#             return None

#     def publish_transformed_point(self, point):
#         # Create a Point message with the transformed coordinates
#         transformed_point_msg = Point()
#         transformed_point_msg.x = point.x
#         transformed_point_msg.y = point.y
#         transformed_point_msg.z = point.z
        
#         # Publish the transformed coordinates to the /transformed_cord topic
#         self.transformed_cord_pub.publish(transformed_point_msg)
#         rospy.loginfo("Transformed coordinates published.")

# if __name__ == "__main__":
#     try:
#         transformer = ObjectTransformer()
#         rospy.spin()  # Keep the node running
#     except rospy.ROSInterruptException:
#         pass

import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Point

class ObjectTransformer:
    def __init__(self):
        rospy.init_node('object_transformer', anonymous=True)
        
        # Subscriber to the /error_xy_meter topic (x, y, z points)
        self.error_xy_meter_sub = rospy.Subscriber('/error_xy_meter', Point, self.xy_callback)
        
        # Publisher to the /transformed_cord topic (transformed x, y, z points)
        self.transformed_cord_pub = rospy.Publisher('/transformed_cord', Point, queue_size=10)

    def xy_callback(self, data):
        # Extract x, y, z from the Point message
        detected_point_camera = [-data.x, data.y, -data.z]
        
        # Perform the transformation to base_link
        transformed_point = self.transform_point(detected_point_camera)
        
        # Publish the transformed point
        if transformed_point:
            self.publish_transformed_point(transformed_point)

    def transform_point(self, point_in_camera_frame):
        # Extract x, y, z from the camera frame point
        x, y, z = point_in_camera_frame
        
        # Define the translation vector from camera_frame_optical to base_link
        translation = np.array([0.29, 0, 0.06])  # Confirm this is correct
        
        # Define the quaternion for rotation
        quaternion = [-0.5, -0.5, 0.500002, 0.499998]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tft.euler_from_quaternion(quaternion)
        rospy.loginfo(f"Euler angles (roll, pitch, yaw): {euler}")
        
        # Create a rotation matrix from Euler angles
        rotation_matrix = tft.quaternion_matrix(quaternion)[:3, :3]  # Extract the rotation part
        
        # Create the point as a column vector
        point = np.array([x, y, z])
        
        # Log the original point and rotation matrix
        rospy.loginfo(f"Original point: {point}, Rotation matrix: {rotation_matrix}, Translation: {translation}")
        
        # Apply the transformation (Rotation + Translation)
        transformed_point = np.dot(rotation_matrix, point) + translation
        
        # Log the transformed point
        rospy.loginfo(f"Transformed point before creating msg: {transformed_point}")
        
        # Convert the transformed point to a ROS Point message
        transformed_point_msg = Point()
        transformed_point_msg.x = transformed_point[0]
        transformed_point_msg.y = transformed_point[1]
        transformed_point_msg.z = transformed_point[2]
        
        return transformed_point_msg

    def publish_transformed_point(self, point):
        """
        Publishes the transformed point to the /transformed_cord topic.
        """
        # Publish the transformed coordinates to the /transformed_cord topic
        self.transformed_cord_pub.publish(point)
        rospy.loginfo(f"Transformed coordinates published: x={point.x}, y={point.y}, z={point.z}")

if __name__ == "__main__":
    try:
        transformer = ObjectTransformer()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
