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
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.1
        self.dist_ld =0.7

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.max_velocity = 10

        self.robot_theta = 0.0
        self.width = 0.45
        self.time_values = []
        self.error_values = []

        self.published_velocity = Int8MultiArray()

        self.waypoints =  [(0,0),(0,0.1),(0,0.2),(0,0.3),(0,0.4),(0,0.5),(0.0,0.6),(0.0,0.7),(0,0.9),(0.0,1),(0,1.1),(0,1.2),(0,1.3),(0,1.4),(0,1.5),(0,1.6),(0,1.7),(0,1.8),(0,1.9), (0,2)]
        # self.waypoints = [(point[1], 0) for point in self.waypoints]

        # Setup matplotlib for plotting
        self.waypoints_plot = None
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))  # Single plot with one axis
        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints and Robot Path')
        self.ax1.set_xlim(-5, 5)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-5, 5)  # Set y-axis limits from -10 to 10
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
        e_longitudinal = 0.0
        e_lateral = 0.0

        # Longitudinal control (using PID)
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.currentx, self.currenty)) - np.array(goal_point))
        
        if distance_to_goal > 0.05:
            e_longitudinal = distance_to_goal
            self.integral += e_longitudinal * self.dt
            
            # Check if error_values list is empty
            if self.error_values:
                derivative = (e_longitudinal - self.error_values[-1]) / self.dt
            else:
                derivative = 0.0  # Default derivative value if list is empty

            action_longitudinal = self.kp * e_longitudinal + self.ki * self.integral + self.kd * derivative
            throttle_output = self.max_velocity * math.tanh(action_longitudinal)
            self.error_values.append(e_longitudinal)
        else:
            throttle_output = 0.0
            self.error_values.append(0.0)

        # Lateral control (using PID)
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        
        if lookahead_point is not None:
            e_lateral = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            self.integral += e_lateral * self.dt
            
            # Check if error_values list is empty
            if self.error_values:
                derivative = (e_lateral - self.error_values[-1]) / self.dt
            else:
                derivative = 0.0  # Default derivative value if list is empty

            action_lateral = self.kp * e_lateral + self.ki * self.integral + self.kd * derivative
            steering_output = math.atan2(2.0 * self.width * math.sin(action_lateral), self.dist_ld)

            # Apply steering_output to control left and right wheel velocities
            Vr = throttle_output * (1 - steering_output / (2 * self.width))
            Vl = throttle_output * (1 + steering_output / (2 * self.width))

            Vr = min(max(Vr, -self.max_velocity), self.max_velocity)
            Vl = min(max(Vl, -self.max_velocity), self.max_velocity)

            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)

            print('Longitudinal Error:', e_longitudinal, ' Lateral Error:', e_lateral)
            print('Throttle Output:', throttle_output, ' Steering Output:', steering_output)
        else:
            self.velocitylf_publisher.publish(0.0)
            self.velocityrf_publisher.publish(0.0)

        return throttle_output


    def find_lookahead_point(self, robot_position):
        candidate_lookahead_points = []
        max_index = -1

        for i, waypoint in enumerate(self.waypoints):
                distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))

                if distance_to_robot < self.dist_ld and i > max_index:
                        candidate_lookahead_points = [waypoint]
                        max_index = i

        if not candidate_lookahead_points:
                return None  # No valid lookahead point found
        
        # Find the index of the candidate with the maximum distance to the goal
        max_distance_index = np.argmax(distance_to_robot)

        # Select the lookahead point with the maximum distance to the goal
        lookahead_point = candidate_lookahead_points[max_distance_index]

        return lookahead_point

    def purePursuit(self):
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))

        if lookahead_point is not None:
            alpha = math.atan2((lookahead_point[1] - self.currenty), (lookahead_point[0] - self.currentx))
            L = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            theta = alpha - self.robot_theta
            dx = L * math.cos(theta)

            throttle_output=self.pidController()

            Vr = throttle_output * (1 - self.width * dx / (L * L))
            Vl = throttle_output * (1 + self.width * dx / (L * L))

            Vr = min(max(Vr, -self.max_velocity), self.max_velocity)
            Vl = min(max(Vl, -self.max_velocity), self.max_velocity)

            print('Right: ', Vr, ' Left: ', Vl)

            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)
                    
            # Plot rover position
            self.plot_rover_position()
            # Give time for plot to update
            plt.pause(0.001)

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
            x.pidController()
    except rospy.ROSInterruptException:
        pass