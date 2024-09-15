#!/usr/bin/env python2.7
import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult

result_goal_reached = False
status_goal_reached = False
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


def status_callback(msg):
    global result_goal_reached, status_goal_reached
    global cmd_vel_pub

    # Check if any of the statuses are 'Goal reached'
    for status in msg.status_list:
        if status.status == 3:
            status_goal_reached = True
        elif status.status == 1:
            status_goal_reached = False
 

    if status_goal_reached and result_goal_reached:
        zero_vel = Twist()
        zero_vel.linear.x = 0.0
        zero_vel.angular.z = 0.0
        cmd_vel_pub.publish(zero_vel)
        rospy.loginfo("Publishing Zero")


    if not status_goal_reached:
        result_goal_reached = False
        rospy.loginfo("lower result_goal_reached")


def result_callback(msg):
    global result_goal_reached
    if msg.status.status == 3:
        result_goal_reached = True
        rospy.loginfo("Goal reached, stopping the robot.")

def listener():
    rospy.init_node('goal_status_listener', anonymous=True)
    
    # Create a publisher for the cmd_vel topic
    
    # Create a subscriber for the move_base/status topic
    rospy.Subscriber('move_base/status', GoalStatusArray, status_callback)
    # Subscriber for /move_base/result
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)
        
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()



    except rospy.ROSInterruptException:
        pass
