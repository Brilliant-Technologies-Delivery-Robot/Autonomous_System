#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

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
    data.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
    # print(data.orientation.x, data.orientation.y, euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[2])
    imu_pub.publish(data)

def odom_callback(data):
    # print odom orientation data in euler angles (degrees)
    orientation = data.pose.pose.orientation
    euler_angles = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    print("Yaw: ", euler_angles[2] * 180 / 3.14159265359)

while not rospy.is_shutdown():
    rospy.init_node('imu_normalizer', anonymous=True)

    imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, odom_callback)

    rospy.spin()
