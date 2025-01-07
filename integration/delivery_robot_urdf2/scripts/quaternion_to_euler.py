#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import Imu
import tf
import math

def imu_callback(msg):
    # Extract the quaternion from the IMU message
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )

    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    roll = euler[0] * (180.0 / math.pi)   # Convert to degrees
    pitch = euler[1] * (180.0 / math.pi)  # Convert to degrees
    yaw = euler[2] * (180.0 / math.pi)    # Convert to degrees

    # Print the Euler angles in degrees
   #  rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw)

def main():
    # Initialize the ROS node
    rospy.init_node('imu_quaternion_to_euler', anonymous=True)

    # Subscribe to the /imu_data topic
    rospy.Subscriber("/imu_data", Imu, imu_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
