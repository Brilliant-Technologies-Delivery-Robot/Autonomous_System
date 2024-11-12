#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

latest_odom = None

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
    global latest_odom

    normalized_quaternion = normalize_quaternion(data.orientation)
    data.orientation = normalized_quaternion
    data.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
    imu_pub.publish(data)

    euler_angles = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    yaw_pub.publish(euler_angles[2])

    if latest_odom is not None:
        latest_odom.header.stamp = rospy.Time.now()
        odom_pub.publish(latest_odom)


def odom_callback(odom_msg):
    global latest_odom
    latest_odom = odom_msg


while not rospy.is_shutdown():
    rospy.init_node('imu_normalizer', anonymous=True)

    imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.Subscriber('/odom_raw', Odometry, odom_callback)

    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    yaw_pub = rospy.Publisher('/yaw', Float64, queue_size=10)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    rospy.spin()
