#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

def move_to_goal(x, y, z, qx, qy, qz, qw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position = Point(x, y, z)
    goal.target_pose.pose.orientation = Quaternion(qx, qy, qz, qw)

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('send_goals_py')

        goals = [
            (1.229, 2.619, 0.0, 0.0, 0.0, 0.636, 0.77166),
            (1.8, 5.38, 0.0, 0.0, 0.0, -0.7, 0.618),
            (-1.099, 1.0679, 0.0, 0.0, 0.0, 0.99127, 0.1317)
        ]

        for goal in goals:
            result = move_to_goal(*goal)
            if result:
                rospy.loginfo("Goal execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
