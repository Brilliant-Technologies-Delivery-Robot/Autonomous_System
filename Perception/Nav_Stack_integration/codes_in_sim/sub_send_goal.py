#!/usr/bin/env python3

# import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Point
# from tf.transformations import quaternion_from_euler

# class SubSendGoal:
#     def __init__(self) -> None:
#         rospy.init_node('send_nav_goal_node', anonymous=True)

#         # Set up the SimpleActionClient for the 'move_base' action server
#         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

#         # Wait for the action server to start
#         rospy.loginfo("Waiting for move_base action server...")
#         self.client.wait_for_server()
#         rospy.loginfo("Connected to move_base server")

#         # Subscribe to the '/goal_to_nav_stack' topic to get goal coordinates
#         rospy.Subscriber('/goal_to_nav_stack', Point, self.goal_data)

#         # Initialize attributes
#         self.goal_received = False
#     def goal_data(self, data):
#         # Process only the first goal received
#         if self.goal_received:
#             rospy.loginfo("Goal already received, ignoring new goal.")
#             return

#         goal_x = data.x 
#         goal_y = data.y

#         rospy.loginfo("Received goal coordinates: x = %.2f, y = %.2f", goal_x, goal_y)

#         # Validate the coordinates before setting the goal
#         if goal_x == 0 and goal_y == 0:
#             rospy.logwarn("Invalid goal received (0, 0), ignoring.")
#             return

#         # Mark the goal as received
#         self.goal_received = True

#         # Set and send the goal
#         self.set_goal(goal_x, goal_y)

#     def set_goal(self, x, y):
#         self.x = x
#         self.y = y
#         self.send_nav_goal()

#     def send_nav_goal(self, frame_id='map'):
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = frame_id
#         goal.target_pose.header.stamp = rospy.Time.now()

#         # Use the coordinates set by the set_goal method
#         goal.target_pose.pose.position.x = self.x
#         goal.target_pose.pose.position.y = self.y
#         goal.target_pose.pose.position.z = 0.0  # Assuming no vertical movement

#         # Set the goal orientation (facing forward)
#         goal.target_pose.pose.orientation.w = 1.0
#         goal.target_pose.pose.orientation.x = 0.0
#         goal.target_pose.pose.orientation.y = 0.0
#         goal.target_pose.pose.orientation.z = 0.0

#         # Send the goal
#         rospy.loginfo("Sending goal: x = %.2f, y = %.2f", self.x, self.y)
#         self.client.send_goal(goal)

#         # Wait for the robot to reach the goal
#         if self.client.wait_for_result():
#             rospy.loginfo("Goal reached, shutting down node.")
#             rospy.signal_shutdown("Goal reached")  # Shutdown after reaching goal
#         else:
#             rospy.logwarn("Failed to reach the goal.")

# def main():
#     # Initialize the goal sender object
#     goal_sender = SubSendGoal()
#     rospy.spin()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         rospy.logerr("ROS Interrupt Exception")




import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from math import sqrt

class SubSendGoal:
    def __init__(self) -> None:
        rospy.init_node('send_nav_goal_node', anonymous=True)

        # Set up the SimpleActionClient for the 'move_base' action server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Wait for the action server to start
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Subscribe to the '/goal_to_nav_stack' topic to get goal coordinates
        rospy.Subscriber('/goal_to_nav_stack', Point, self.goal_data)

        # Initialize attributes
        self.threshold = 0.3  # Minimum distance between consecutive goals
        self.last_goal = None  # Last goal sent as (x, y)
        self.waypoints = []  # Queue of valid waypoints

        # State to prevent sending multiple goals at once
        self.is_goal_active = False

    def goal_data(self, data):
        # Get new goal coordinates
        new_goal = (data.x, data.y)

        # Check if this is the first goal or far enough from the last goal
        if self.last_goal is None or self.is_far_enough(self.last_goal, new_goal):
            rospy.loginfo("Received valid goal: x = %.2f, y = %.2f", new_goal[0], new_goal[1])
            self.waypoints.append(new_goal)  # Add new goal to the queue

            # Start sending goals if not already active
            if not self.is_goal_active:
                self.process_next_goal()
        else:
            rospy.logwarn("Goal too close to the previous one, ignoring: x = %.2f, y = %.2f", new_goal[0], new_goal[1])

    def is_far_enough(self, last_goal, new_goal):
        # Calculate the Euclidean distance between the last and new goals
        distance = sqrt((new_goal[0] - last_goal[0])**2 + (new_goal[1] - last_goal[1])**2)
        return distance > self.threshold

    def process_next_goal(self):
        if not self.waypoints:
            rospy.loginfo("No more waypoints to process.")
            self.is_goal_active = False
            return

        # Pop the next goal from the queue
        self.is_goal_active = True
        next_goal = self.waypoints.pop(0)
        self.last_goal = next_goal  # Update the last goal
        self.set_goal(next_goal[0], next_goal[1])

    def set_goal(self, x, y):
        self.x = x
        self.y = y
        self.send_nav_goal()

    def send_nav_goal(self, frame_id='map'):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        # Use the coordinates set by the set_goal method
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = 0.0  # Assuming no vertical movement

        # Set the goal orientation (facing forward)
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0

        # Send the goal
        rospy.loginfo("Sending goal: x = %.2f, y = %.2f", self.x, self.y)
        self.client.send_goal(goal)

        # Wait for the robot to reach the goal
        self.client.wait_for_result()
        if self.client.get_result():
            rospy.loginfo("Goal reached.")
            self.process_next_goal()  # Move to the next waypoint
        else:
            rospy.logwarn("Failed to reach the goal.")
            self.is_goal_active = False

def main():
    # Initialize the goal sender object
    goal_sender = SubSendGoal()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")