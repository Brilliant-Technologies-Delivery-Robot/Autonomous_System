#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64,Float64MultiArray, Int8MultiArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt

class Control:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/nav_action/supervised', Int8MultiArray, queue_size=10)

        self.velocitylf_publisher = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.velocityrf_publisher = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.pose = ModelStates()
        self.throttle_output=Float64()

        self.rate = rospy.Rate(10)
        self.kp = 2.5
        self.ki = 3
        self.kd = 0.0
        self.dist_ld =1.5

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.current_waypoint_index = 0

        self.max_velocity = 10

        self.robot_theta = 0.0
        self.width = 0.45
        self.time_values = []
        self.error_values = []

        self.published_velocity = Int8MultiArray()

        # Define the original waypoints
        # self.waypoints = [(0, 0), (0, 0.1), (0, 0.2), (0, 0.3), (0, 0.4), (0, 0.5), (0.0, 0.6), (0.0, 0.7), (0, 0.9), (0.0, 1), 
        #                 (0, 1.1), (0, 1.2), (0, 1.3), (0, 1.4), (0, 1.5), (0, 1.6), (0, 1.7), (0, 1.8), (0, 1.9), (0, 2)]

        # self.waypoints = [(0, 0), (0.1, 0.1), (0.2, 0.2), (0.3, 0.3), (0.4, 0.4), (0.5, 0.5), (0.6, 0.6), (0.7, 0.7), (0.9, 0.9), (1.0, 1), 
        #                 (1.1, 1.1), (1.2, 1.2), (1.3, 1.3), (1.4, 1.4), (1.5, 1.5), (1.6, 1.6), (1.7, 1.7), (1.8, 1.8), (1.9, 1.9), (2, 2)]

            # self.waypoints = [ 
            # #     #  (0, 0),(0.5, 0.2), (1, 0.5),(1.5, 1), (2, 2), (2,4),(3.5, 4),(4,4),(4.5, 4), (5, 4.5),(6,6), (7, 6.5), (8, 7.5),(9, 8.5),(9.5, 9)]
            #         (0, 0),
            #     (0.025, 0.01),
            #     (0.05, 0.02),
            #     (0.075, 0.03),
            #     (0.1, 0.04),
            #     (0.125, 0.05),
            #     (0.15, 0.06),
            #     (0.175, 0.07),
            #     (0.2, 0.08),
            #     (0.225, 0.09),
            #     (0.25, 0.1),
            #     (0.275, 0.11),
            #     (0.3, 0.12),
            #     (0.325, 0.13),
            #     (0.35, 0.14),
            #     (0.375, 0.15),
            #     (0.4, 0.16),
            #     (0.425, 0.17),
            #     (0.45, 0.18),
            #     (0.475, 0.19),
            #     (0.5, 0.2),
            #     (0.55, 0.23),
            #     (0.6, 0.26),
            #     (0.65, 0.29),
            #     (0.7, 0.32),
            #     (0.75, 0.35),
            #     (0.8, 0.38),
            #     (0.85, 0.41),
            #     (0.9, 0.44),
            #     (0.95, 0.47),
            #     (1, 0.5),
            #     (1.05, 0.55),
            #     (1.1, 0.6),
            #     (1.15, 0.65),
            #     (1.2, 0.7),
            #     (1.25, 0.75),
            #     (1.3, 0.8),
            #     (1.35, 0.85),
            #     (1.4, 0.9),
            #     (1.45, 0.95),
            #     (1.5, 1),
            #     (1.55, 1.2),
            #     (1.6, 1.4),
            #     (1.65, 1.6),
            #     (1.7, 1.8),
            #     (1.75, 2),
            #     (1.8, 2.2),
            #     (1.85, 2.4),
            #     (1.9, 2.6),
            #     (1.95, 2.8),
            #     (2, 3),
            #     (2.05, 3.2),
            #     (2.1, 3.4),
            #     (2.15, 3.6),
            #     (2.2, 3.8),
            #     (2.25, 4),
            #     (2.3, 4),
            #     (2.35, 4),
            #     (2.4, 4),
            #     (2.45, 4),
            #     (2.5, 4),
            #     (2.55, 4),
            #     (2.6, 4),
            #     (2.65, 4),
            #     (2.7, 4),
            #     (2.75, 4),
            #     (2.8, 4),
            #     (2.85, 4),
            #     (2.9, 4),
            #     (2.95, 4),
            #     (3, 4),
            #     (3.05, 4),
            #     (3.1, 4),
            #     (3.15, 4),
            #     (3.2, 4),
            #     (3.25, 4),
            #     (3.3, 4),
            #     (3.35, 4),
            #     (3.4, 4),
            #     (3.45, 4),
            #     (3.5, 4),
            #     (3.55, 4),
            #     (3.6, 4),
            #     (3.65, 4),
            #     (3.7, 4),
            #     (3.75, 4),
            #     (3.8, 4),
            #     (3.85, 4),
            #     (3.9, 4),
            #     (3.95, 4),
            #     (4, 4),
            #     (4.05, 4),
            #     (4.1, 4),
            #     (4.15, 4),
            #     (4.2, 4),
            #     (4.25, 4),
            #     (4.3, 4),
            #     (4.35, 4),
            #     (4.4, 4),
            #     (4.45, 4),
            #     (4.5, 4),
            #     (4.55, 4.1),
            #     (4.6, 4.2),
            #     (4.65, 4.3),
            #     (4.7, 4.4),
            #     (4.75, 4.5),
            #     (4.8, 4.6),
            #     (4.85, 4.7),
            #     (4.9, 4.8),
            #     (4.95, 4.9),
            #     (5, 5),
            #     (5.1, 5.1),
            #     (5.2, 5.2),
            #     (5.3, 5.3),
            #     (5.4, 5.4),
            #     (5.5, 5.5),
            #     (5.6, 5.6),
            #     (5.7, 5.7),
            #     (5.8, 5.8),
            #     (5.9, 5.9),
            #     (6, 6),
            #     (6.1, 6.05),
            #     (6.2, 6.1),
            #     (6.3, 6.15),
            #     (6.4, 6.2),
            #     (6.5, 6.25),
            #     (6.6, 6.3),
            #     (6.7, 6.35),
            #     (6.8, 6.4),
            #     (6.9, 6.45),
            #     (7, 6.5),
            #     (7.1, 6.55),
            #     (7.2, 6.6),
            #     (7.3, 6.65),
            #     (7.4, 6.7),
            #     (7.5, 6.75),
            #     (7.6, 6.8),
            #     (7.7, 6.85),
            #     (7.8, 6.9),
            #     (7.9, 6.95),
            #     (8, 7),
            #     (8.1, 7.05),
            #     (8.2, 7.1),
            #     (8.3, 7.15),
            #     (8.4, 7.2),
            #     (8.5, 7.25),
            #     (8.6, 7.3),
            #     (8.7, 7.35),
            #     (8.8, 7.4),
            #     (8.9, 7.45),
            #     (9, 7.5),
            #     (9.05, 7.55),
            #     (9.1, 7.6),
            #     (9.15, 7.65),
            #     (9.2, 7.7),
            #     (9.25, 7.75),
            #     (9.3, 7.8),
            #     (9.35, 7.85),
            #     (9.4, 7.9),
            #     (9.45, 7.95),
            #     (9.5, 8),
            #     (9.55, 8.05),
            #     (9.6, 8.1),
            #     (9.65, 8.15),
            #     (9.7, 8.2),
            #     (9.75, 8.25),
            #     (9.8, 8.3),
            #     (9.85, 8.35),
            #     (9.9, 8.4),
            #     (9.95, 8.45),
            #     (10, 8.5),
            #     (10.025, 8.525),
            #     (10.05, 8.55),
            #     (10.075, 8.575),
            #     (10.1, 8.6),
            #     (10.125, 8.625),
            #     (10.15, 8.65),
            #     (10.175, 8.675),
            #     (10.2, 8.7),
            #     (10.225, 8.725),
            #     (10.25, 8.75),
            #     (10.275, 8.775),
            #     (10.3, 8.8),
            #     (10.325, 8.825),
            #     (10.35, 8.85),
            #     (10.375, 8.875),
            #     (10.4, 8.9),
            #     (10.425, 8.925),
            #     (10.45, 8.95),
            #     (10.475, 8.975),
            #     (10.5, 9),

            # ]
                        # (0, 0),
                        # (0.125, 0.425),
                        # (0.25, 0.85),
                        # (0.375, 1.275),
                        # (0.5, 1.7),
                        # (0.625, 1.85),
                        # (0.75, 2),
                        # (0.875, 2.15),
                        # (1, 2.3),
                        # (1.125, 2.35),
                        # (1.25, 2.4),
                        # (1.375, 2.45),
                        # (1.5, 2.5),
                        # (1.625, 2.55),
                        # (1.75, 2.6),
                        # (1.875, 2.65),
                        # (2, 2.7),
                        # (2.125, 2.7),
                        # (2.25, 2.7),
                        # (2.375, 2.7),
                        # (2.5, 2.7),
                        # (2.625, 2.7),
                        # (2.75, 2.7),
                        # (2.875, 2.7),
                        # (3, 2.7),
                        # (3.125, 2.65),
                        # (3.25, 2.6),
                        # (3.375, 2.55),
                        # (3.5, 2.5),
                        # (3.625, 2.45),
                        # (3.75, 2.4),
                        # (3.875, 2.35),
                        # (4, 2.3),
                        # (4.125, 2.25),
                        # (4.25, 2.2),
                        # (4.375, 2.15),
                        # (4.5, 2.1),
                        # (4.625, 1.9),
                        # (4.75, 1.7),
                        # (4.875, 1.275),(4.9,1.175),(4.95,1),(5,0.75),(5,0.6),(5,0.5),(5,0.25),
                        # (5, 0)
                        # ]

        self.waypoints= [
                (0, 0),         # Starting point
                (2, 0.5),       # Intermediate point
                (3, 1),         # Intermediate point
                (4, 2),         # Intermediate point
                (5, 2.5),       # Intermediate point
                (5, 3),         # Turn into a room or a different corridor
                (7, 3),         # Intermediate point
                (8, 3.5),       # Intermediate point
                (9, 4),         # Intermediate point
                (9, 5),         # Intermediate point
                (9, 6),         # Intermediate point
                (8, 6),         # Intermediate point
                (7, 7),         # Intermediate point
                (6, 7),         # Intermediate point
                (5, 7.5),       # Intermediate point
                (5, 8),         # Intermediate point
                (4, 8),         # Intermediate point
                (3, 8),         # Intermediate point
                (2, 8),         # Intermediate point
                (1, 8),         # Intermediate point
                (1, 7),         # Intermediate point
                (0, 7),         # Intermediate point
                (0, 6),         # Intermediate point
                (0, 5),         # Navigate towards an intersection or different area
                (1, 5),         # Intermediate point
                (2, 5),         # Intermediate point
                (2, 4),         # Intermediate point
                (3, 4),         # Intermediate point
                (3, 3),         # Intermediate point
                (4, 3),         # Intermediate point
                (4, 2),         # Intermediate point
                (3, 2),         # Intermediate point
                (2, 2),         # Intermediate point
                (1, 2),         # Intermediate point
                (0, 2),         # Intermediate point
                (0, 1),         # Intermediate point
                (1, 1),         # Intermediate point
                (2, 1),         # Intermediate point
                (3, 1),         # Move further into the room or corridor
                (4, 1),         # Intermediate point
                (5, 1),         # Intermediate point
                (5, 2),         # Navigate towards an intersection or different area
                (6, 2),         # Intermediate point
                (7, 2),         # Intermediate point
                (8, 2),         # Intermediate point
                (8, 3),         # Move further into the hospital area
                (7, 3),         # Intermediate point
                (6, 3),         # Intermediate point
                (5, 3),         # Intermediate point
                (5, 4),         # Intermediate point
                (4, 4),         # Intermediate point
                (3, 4),         # Intermediate point
                (3, 5),         # Intermediate point
                (2, 5),         # Intermediate point
                (1, 5),         # Intermediate point
                (1, 4),         # Intermediate point
                (0, 4),         # Intermediate point
                (0, 3),         # Intermediate point
                (1, 3),         # Intermediate point
                (2, 3),         # Intermediate point
                (2, 2),         # Intermediate point
                (3, 2),         # Intermediate point
                (4, 2),         # Intermediate point
                (5, 2),         # Intermediate point
                (6, 2),         # Intermediate point
                (6, 3),         # Intermediate point
                (7, 3),         # Intermediate point
                (8, 3),         # Intermediate point
                (9, 3),         # Intermediate point
                (9, 4),         # Intermediate point
                (9, 5),         # Intermediate point
                (8, 5),         # Intermediate point
                (7, 5),         # Intermediate point
                (6, 5),         # Intermediate point
                (5, 5),         # Intermediate point
                (4, 5),         # Intermediate point
                (3, 5),         # Intermediate point
                (2, 5),         # Intermediate point
                (1, 5),         # Navigate towards an intersection or different area
                (1, 6),         # Intermediate point
                (2, 6),         # Intermediate point
                (3, 6),         # Intermediate point
                (4, 6),         # Intermediate point
                (5, 6),         # Intermediate point
                (6, 6),         # Intermediate point
                (7, 6),         # Intermediate point
                (8, 6),         # Intermediate point
                (9, 6),         # Intermediate point
                (9, 7),         # Intermediate point
                (8, 7),         # Intermediate point
                (7, 7),         # Intermediate point
                (6, 7),         # Intermediate point
                (5, 7),         # Intermediate point
                (4, 7),         # Intermediate point
                (3, 7),         # Intermediate point
                (2, 7),         # Intermediate point
                (1, 7),         # Intermediate point
                (1, 6),         # Intermediate point
                (1, 5),         # Navigate towards an intersection or different area
                (1, 4),         # Intermediate point
                (1, 3),         # Intermediate point
                (1, 2),         # Intermediate point
                (2, 2),         # Intermediate point
                (2, 3),         # Intermediate point
                (3, 3),         # Intermediate point
                (3, 2),         # Intermediate point
                (4, 2),         # Intermediate point
                (5, 2),         # Intermediate point
                (5, 1),         # Intermediate point
                (5, 0),         # Intermediate point
                (5, 0)          # End point (same as starting point)
            ]

        # Transform the waypoints
        # self.waypoints = [(point[1], 0) for point in self.waypoints]

        # # Multiply each coordinate of each waypoint by -1
        # self.waypoints = [(-x, -y) for x, y in self.waypoints]


        # Setup matplotlib for plotting
        self.waypoints_plot = None
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))  # Single plot with one axis
        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints and Robot Path')
        self.ax1.set_xlim(-5, 11)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-5, 11)  # Set y-axis limits from -10 to 10
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_plot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robot_position_plot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)
        # Add a plot for the robot's path
        self.past_positions_x = []
        self.past_positions_y = []
        self.robot_path_plot, = self.ax1.plot([], [], 'r-', label='Robot Path')  # Robot path plot
        self.ax1.legend()
        plt.tight_layout()
    
        self.update_waypoints_plot()


    def update_pose(self, data:ModelStates):
        self.pose = data
        self.currentx = self.pose.pose[1].position.x 
        self.currenty = self.pose.pose[1].position.y
        orientation = self.pose.pose[1].orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def pidController(self):
        e = 0.0
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.currentx, self.currenty)) - np.array(goal_point))

        if lookahead_point is not None and distance_to_goal > 0.1:
            e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            print("Selected Lookahead Point:", lookahead_point)
            e_past = 0
            if e > 0.05:
                self.integral += e * self.dt
                derivative = (e - e_past) / self.dt
                action = self.kp * e + self.ki * self.integral + self.kd * derivative
                self.throttle_output = self.max_velocity * math.tanh(action)
                e_past = e
                print('Error = ', e)
            else:
                self.throttle_output = 0.0     
        else:
            self.throttle_output = 0.0

        rospy.loginfo('Error = %f', e)

        return self.throttle_output

    def purePursuit(self):
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))

        if lookahead_point is not None:
            alpha = math.atan2((lookahead_point[1] - self.currenty), (lookahead_point[0] - self.currentx))
            L = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            theta = alpha - self.robot_theta
            
            # Ensure theta is within -pi to pi
            if theta > math.pi:
                theta -= 2 * math.pi
            elif theta < -math.pi:
                theta += 2 * math.pi

            # Calculate curvature
            kappa = 2 * math.sin(theta) / L
            
            # Given throttle output (V)
            V = self.pidController()  # Extract the float value
            
            # Calculate wheel velocities
            Vr = V * (1 - (self.width / 2) * kappa)
            Vl = V * (1 + (self.width / 2) * kappa)

            # Clamp the velocities to max_velocity
            Vr = min(max(Vr, -self.max_velocity), self.max_velocity)
            Vl = min(max(Vl, -self.max_velocity), self.max_velocity)

            print('Right: ', Vr, ' Left: ', Vl)

            self.velocitylf_publisher.publish(Vr)
            self.velocityrf_publisher.publish(Vl)

            # Plot rover position
            self.plot_rover_position()
            # Give time for plot to update
            plt.pause(0.001)
        # else:
        #     print("No valid lookahead point found.")



    def find_lookahead_point(self, robot_position):
        candidate_lookahead_points = []
        max_index = -1

        for i, waypoint in enumerate(self.waypoints):
                distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))

                if distance_to_robot < self.dist_ld and i > max_index:
                        candidate_lookahead_points = [waypoint]
                        max_index = i

        if not candidate_lookahead_points:
            return self.waypoints[-1]
        
        # Find the index of the candidate with the maximum distance to the goal
        max_distance_index = np.argmax(distance_to_robot)

        # Select the lookahead point with the maximum distance to the goal
        lookahead_point = candidate_lookahead_points[max_distance_index]

        return lookahead_point
    

    def plot_rover_position(self):
        # Append the current position to the past positions
        self.past_positions_x.append(self.currentx)
        self.past_positions_y.append(self.currenty)

        # Update the robot path plot
        self.robot_path_plot.set_data(self.past_positions_x, self.past_positions_y)
        self.robot_position_plot.set_data([self.currentx], [self.currenty])  # Update the current position plot

    def update_waypoints_plot(self):
        self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
        self.waypoints_plot.set_data(self.waypoints_x, self.waypoints_y)

if __name__ == '__main__':
    try:
        x = Control()
        while not rospy.is_shutdown():
            if len(x.waypoints) > 0:
                # x.pidController()
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass