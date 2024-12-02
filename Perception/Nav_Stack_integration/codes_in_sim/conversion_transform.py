#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import Point, PointStamped
import tf.transformations as tft

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
