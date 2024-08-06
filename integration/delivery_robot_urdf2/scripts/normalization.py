#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def normalize_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    norm = (x**2 + y**2 + z**2 + w**2)**0.5

    if norm == 0:
        rospy.logwarn("Quaternion norm is zero. Skipping normalization.")
        return quaternion

    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm

    return quaternion

def imu_callback(data):
    normalized_quaternion = normalize_quaternion(data.orientation)
    data.orientation = normalized_quaternion
    data.orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
    print(data.orientation.x, data.orientation.y, euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[2])
    imu_pub.publish(data)

while not rospy.is_shutdown():
    rospy.init_node('imu_normalizer', anonymous=True)

    imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)

    rospy.spin()
