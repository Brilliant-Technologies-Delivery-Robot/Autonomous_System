#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# File path
file_path = "/home/seif/delivery_robot/src/Autonomous_System/navigation_control/scripts/planning/data.txt"

def tuple_list_publisher():
    rospy.init_node('tuple_list_publisher', anonymous=True)
    pub = rospy.Publisher('tuple_list_topic', Path, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        x_way_points_list = []
        y_way_points_list = []

        # Read all waypoints from the file
        with open(file_path, 'r') as file:
            for line in file:
                elements = line.strip().split('\n')
                for line in elements:
                    elements = line.strip()[1:-1].split(',')
                    x = float(elements[0])
                    y = float(elements[1])
                    x_way_points_list.append(x)
                    y_way_points_list.append(y)

        # Create a Path message with all waypoints
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in zip(x_way_points_list, y_way_points_list):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg.poses.append(pose)

        # Publish the message
        pub.publish(msg)
        rospy.loginfo("Published a list of tuples")

        # Sleep for the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        tuple_list_publisher()
    except rospy.ROSInterruptException:
        pass
