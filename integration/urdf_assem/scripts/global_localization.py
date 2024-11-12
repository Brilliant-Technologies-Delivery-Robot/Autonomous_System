#!/usr/bin/env python2.7

import rospy
from std_srvs.srv import Empty

def call_global_localization():
    rospy.wait_for_service('/global_localization')
    try:
        global_localization = rospy.ServiceProxy('/global_localization', Empty)
        response = global_localization()
        rospy.loginfo("Global localization triggered successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('call_global_localization_node')
    call_global_localization()
