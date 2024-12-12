#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PoseStamped
import math


class SubSendGoal:
    def __init__(self):
        rospy.init_node('send_nav_goal_node', anonymous=True)
        
        # Set up the SimpleActionClient for the 'move_base' action server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
        # Wait for the action server to start
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")
        
        # Set up the TF2 buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscriber for goal point
        self.goal_subscriber = rospy.Subscriber('/goal_to_nav_stack', Point, self.goal_callback)

        # A flag to check when to start navigating
        self.new_goal_received = False
        self.current_goal = None
        self.previous_goal = None
        self.threshold = 0.3  # Distance threshold in meters

    def goal_callback(self, msg):
        """
        Callback for the goal point topic, saves the received goal.
        """
        rospy.loginfo("Received new goal: x = %.2f, y = %.2f", msg.x, msg.y)
        # Convert Point to PoseStamped (no need for orientation or z for 2D)
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Assuming map frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0.0  # No vertical movement
        pose.pose.orientation.w = 1.0  # Default orientation (facing forward)

        self.current_goal = pose
        self.new_goal_received = True

    def calculate_distance(self, goal1, goal2):
        """
        Calculate the distance between two goals in 2D space.
        """
        dx = goal1.pose.position.x - goal2.pose.position.x
        dy = goal1.pose.position.y - goal2.pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def transform_goal(self, map_goal):
        """
        Transform the goal from the 'map' frame to the 'base_link' frame if required.
        """
        try:
            rospy.loginfo("Waiting for transformation from 'map' to 'base_link'")
            self.tf_buffer.can_transform('chassis', 'map', rospy.Time(0), rospy.Duration(5.0))
            transform = self.tf_buffer.lookup_transform('chassis', 'map', rospy.Time(0), rospy.Duration(5.0))
            base_link_goal = tf2_geometry_msgs.do_transform_pose(map_goal, transform)
            return base_link_goal
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform goal: %s", str(e))
            return None

    def send_nav_goal(self, goal):
        """
        Send the navigation goal to the move_base action server.
        """
        # Transform the goal to the 'base_link' frame if necessary
        if goal.header.frame_id != 'base_link':
            transformed_goal = self.transform_goal(goal)
            if not transformed_goal:
                return False
        else:
            transformed_goal = goal

        # Create a MoveBaseGoal message
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = transformed_goal

        # Send the goal to the move_base action server
        rospy.loginfo("Sending goal: x = %.2f, y = %.2f", 
                      transformed_goal.pose.position.x, 
                      transformed_goal.pose.position.y)
        self.client.send_goal(move_base_goal)

        # Wait for the robot to reach the goal
        self.client.wait_for_result()
        if self.client.get_result():
            rospy.loginfo("Goal reached: x = %.2f, y = %.2f", 
                          transformed_goal.pose.position.x, 
                          transformed_goal.pose.position.y)
            return True
        else:
            rospy.logerr("Failed to reach the goal: x = %.2f, y = %.2f", 
                         transformed_goal.pose.position.x, 
                         transformed_goal.pose.position.y)
            return False

    def follow_waypoints(self):
        """
        Keep checking for new goals and navigate to them, with threshold check.
        """
        while not rospy.is_shutdown():
            if self.new_goal_received and self.current_goal:
                if self.previous_goal:
                    # Calculate the distance between the current goal and the previous goal
                    distance = self.calculate_distance(self.current_goal, self.previous_goal)
                    rospy.loginfo("Distance to previous goal: %.2f", distance)

                    if distance <= self.threshold:
                        rospy.loginfo("Goal is too close to the previous goal. Skipping...")
                        self.new_goal_received = False
                        continue

                rospy.loginfo("Navigating to received goal: x = %.2f, y = %.2f", 
                              self.current_goal.pose.position.x, 
                              self.current_goal.pose.position.y)
                if not self.send_nav_goal(self.current_goal):
                    rospy.logerr("Failed to reach the goal: x = %.2f, y = %.2f", 
                                 self.current_goal.pose.position.x, 
                                 self.current_goal.pose.position.y)

                # Update previous goal after sending the current goal
                self.previous_goal = self.current_goal
                self.new_goal_received = False


def main():
    # Initialize the goal sender object
    goal_sender = SubSendGoal()
    goal_sender.follow_waypoints()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
