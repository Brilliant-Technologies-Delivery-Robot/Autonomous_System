#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt


class Control:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.velocitylf_publisher = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.velocityrf_publisher = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)
        
        self.path_subscriber = rospy.Subscriber('tuple_list_topic', Path, self.tuple_list_callback)

        self.vel_pub = rospy.Publisher('vel', Vector3, queue_size=10)
        
        self.pose = ModelStates()
        self.throttle_output = Float64()
        self.vel_msg = Vector3()

        self.rate = rospy.Rate(10)
        self.kp = 20
        self.ki = 1.2
        self.kd = 0.1
        self.dist_ld = 0.2
        self.min_ld = 0.1  # Minimum lookahead distance
        self.max_ld = 1  # Maximum lookahead distance

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.current_waypoint_index = 0

        self.max_velocity = 8

        self.robot_theta = 0.0
        self.width = 0.45
        self.time_values = []
        self.error_values = []
        self.robot_trajectory = []

        self.waypoints = []  # Initialize waypoints to an empty list

        self.goal_reached = False
        self.waiting_for_path = True


        # Initialize plotting
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.waypoints_plot, = self.ax.plot([], [], 'b--', label='Waypoints')
        self.trajectory_plot, = self.ax.plot([], [], 'r-', label='Trajectory')
        self.robot_position_plot, = self.ax.plot([], [], 'r^', markersize=10, label='Robot Position')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()
        self.ax.set_title('Robot Trajectory vs Waypoints')
        self.ax.grid(True)

    def update_pose(self, data):
        self.pose = data
        self.currentx = data.pose[1].position.x
        self.currenty = data.pose[1].position.y
        orientation_q = data.pose[1].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw
        self.robot_trajectory.append((self.currentx, self.currenty))
        
    def tuple_list_callback(self, msg):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.goal_reached = False  # Reset the goal reached flag
        self.waiting_for_path = False  # Indicate that we have received a new path
        # rospy.loginfo(f"Received waypoints: {self.waypoints}")
        self.update_waypoints_plot()

    def update_waypoints_plot(self):
        waypoints_x, waypoints_y = zip(*self.waypoints) if self.waypoints else ([], [])
        self.waypoints_plot.set_data(waypoints_x, waypoints_y)
        self.ax.relim()
        self.ax.autoscale_view()

    def map_velocity(self, velocity):
        # Clamping the value to be within the range -self.max_velocity to self.max_velocity
        velocity = min(max(velocity, -self.max_velocity), self.max_velocity)
        
        if velocity == 0:
            return 64  # Return 64 if the velocity is exactly 0
        elif velocity < 0:
            # Mapping negative values from -self.max_velocity to 0 to the range 32 to 61
            return int(((velocity + self.max_velocity) / self.max_velocity) * (61 - 32) + 32)
        else:
            # Mapping positive values from 0 to self.max_velocity to the range 67 to 95
            return int((velocity / self.max_velocity) * (95 - 67) + 67)

    def purePursuit(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("Goal Reached")
            self.goal_reached = True  # Set the goal reached flag
            self.plot_trajectory()
            return

        lookahead_point = self.find_lookahead_point()

        goal_x, goal_y = self.waypoints[-1]
        distance_to_goal = math.hypot(goal_x - self.currentx, goal_y - self.currenty)
        e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
        
        if e < 0.1:
            rospy.loginfo("Goal reached")
            Vr = 0.0
            Vl = 0.0
            
            self.vel_msg.x = self.map_velocity(Vr)
            self.vel_msg.y = self.map_velocity(Vl)
            self.vel_pub.publish(self.vel_msg)

            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)

            rospy.loginfo(f"Vr: {Vr}, Vl: {Vl}, Mapped Vr: {self.vel_msg.x}, Mapped Vl: {self.vel_msg.y}")

            self.goal_reached = True  # Set the goal reached flag
            self.plot_trajectory()
            return

        angle_to_point = math.atan2(lookahead_point[1] - self.currenty, lookahead_point[0] - self.currentx)
        angle_diff = angle_to_point - self.robot_theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize the angle

        # Adjusting the threshold for rotation based on the distance to the next waypoint
        distance_to_next_waypoint = math.hypot(self.waypoints[self.current_waypoint_index][0] - self.currentx, 
                                                self.waypoints[self.current_waypoint_index][1] - self.currenty)
        
        if abs(angle_diff) > math.pi / 2 and distance_to_next_waypoint > 0.5:  # Adjust the distance threshold as needed
            rospy.loginfo("Waypoint is behind the robot, rotating in place")
            if angle_diff > 0:
                Vr = 2.5
                Vl = -2.5
            else:
                Vr = -2.5
                Vl = 2.5

            self.vel_msg.x = self.map_velocity(Vr)
            self.vel_msg.y = self.map_velocity(Vl)
            self.vel_pub.publish(self.vel_msg)

            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)

            rospy.loginfo(f"Vr: {Vr}, Vl: {Vl}, Mapped Vr: {self.vel_msg.x}, Mapped Vl: {self.vel_msg.y}")
            return

        curvature = 2 * math.sin(angle_diff) / self.dist_ld

        # PID control for velocity
        error = distance_to_goal
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        velocity = min(velocity, self.max_velocity)

        Vr = velocity * (2 + curvature * self.width) / 2
        Vl = velocity * (2 - curvature * self.width) / 2

        Vr = min(max(Vr, -self.max_velocity), self.max_velocity)
        Vl = min(max(Vl, -self.max_velocity), self.max_velocity)

        self.vel_msg.x = self.map_velocity(Vr)
        self.vel_msg.y = self.map_velocity(Vl)

        self.velocitylf_publisher.publish(Vl)
        self.velocityrf_publisher.publish(Vr)

        self.vel_pub.publish(self.vel_msg)
        rospy.loginfo(f"Vr: {Vr}, Vl: {Vl}, Mapped Vr: {self.vel_msg.x}, Mapped Vl: {self.vel_msg.y}")

        self.plot_trajectory()

    def find_lookahead_point(self):
        lookahead_point = None
        ld = self.min_ld
        lookahead_point_found = False

        while ld <= self.max_ld:
            for i in range(self.current_waypoint_index, len(self.waypoints)):
                point = self.waypoints[i]
                distance = math.hypot(point[0] - self.currentx, point[1] - self.currenty)
                if distance >= ld:
                    lookahead_point = point
                    self.current_waypoint_index = i
                    rospy.loginfo(f"Selected lookahead point {lookahead_point} at distance {distance}")
                    lookahead_point_found = True
                    break
            if lookahead_point_found:
                break
            ld += 0.1  # Increment ld only if a lookahead point hasn't been found

        if not lookahead_point:
            lookahead_point = self.waypoints[-1]
            if not self.goal_reached:  # Ensure the message is printed only once
                rospy.loginfo(f"Using final goal at {self.waypoints[-1]} as lookahead point")
        
        return lookahead_point

    def plot_trajectory(self):
        if not self.robot_trajectory:
            return  # Skip plotting if there's no trajectory data

        try:
            trajectory_x, trajectory_y = zip(*self.robot_trajectory)
        except ValueError:
            rospy.logwarn("Mismatch in trajectory x and y lengths, skipping plot")
            return

        if len(trajectory_x) != len(trajectory_y):
            rospy.logwarn("Mismatch in trajectory x and y lengths")
            return

        self.trajectory_plot.set_data(trajectory_x, trajectory_y)
        self.robot_position_plot.set_data([self.currentx], [self.currenty])
        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()

        if self.goal_reached:
            plt.savefig('final_trajectory.png')  # Save the final trajectory plot when the goal is reached



if __name__ == '__main__':
    x = Control()
    while not rospy.is_shutdown():
        if not x.goal_reached:
            x.purePursuit()
            x.plot_trajectory()
        else:
            rospy.sleep(1)  # Sleep to prevent busy-waiting
        plt.pause(0.1)
        rospy.sleep(0.1)

