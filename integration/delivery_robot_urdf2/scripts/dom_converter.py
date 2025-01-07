#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def callback(msg):
    odom = Odometry()
    odom.header = msg.header
    odom.pose = msg.pose
    pub.publish(odom)

rospy.init_node('pose_to_odom')
pub = rospy.Publisher('/robot_pose_ekf/odom_combined_converted', Odometry, queue_size=10)
rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, callback)
rospy.spin()

