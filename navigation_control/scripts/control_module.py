#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Vector3
from nav_msgs.msg import Odometry
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

        # self.path_subscriber = rospy.Subscriber('tuple_list_topic', wp_list, self.tuple_list_callback)  

        self.vel_pub = rospy.Publisher('vel', Vector3, queue_size=10)
    
        
        self.pose = ModelStates()
        self.throttle_output = Float64()
        self.vel_msg = Vector3()

        self.rate = rospy.Rate(10)
        self.kp = 15
        self.ki = 1.2
        self.kd = 0.1
        self.dist_ld = 0.2
        self.min_ld = 0.2 # Minimum lookahead distance
        self.max_ld = 1  # Maximum lookahead distance

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.current_waypoint_index = 0

        self.max_velocity = 10

        self.robot_theta = 0.0
        self.width = 0.45
        self.time_values = []
        self.error_values = []
        self.robot_trajectory = []
        
        # self.waypoints =  [(0,0),(0,0.1),(0,0.2),(0,0.3),(0,0.4),(0,0.5),(0.0,0.6),(0.0,0.7),(0,0.9),(0.0,1),(0,1.1),(0,1.2),(0,1.3),(0,1.4),(0,1.5),(0,1.6),(0,1.7),(0,1.8),(0,1.9), (0,2)]
        # self.waypoints = [(point[1], 0) for point in self.waypoints]
        # self.waypoints = [(-x, -y) for x, y in self.waypoints]

        self.waypoints= [(0,0),(1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(7,1),(7,2),(6,2),(5,2),(4,2),(3,2),(2,2),(1,2),(0.5,2),(0.5,3),(0.5,4),(1,4),(2,4),(4,4), (7,4),(7,6),(5,10),(3,11),(0,11)]
 

        # self.waypoints= [
        #         (0, 0),         # Starting point
        #         (2, 0.5),       # Intermediate point
        #         (3, 1),         # Intermediate point
        #         (4, 2),         # Intermediate point
        #         (5, 2.5),       # Intermediate point
        #         (5, 3),         # Turn into a room or a different corridor
        #         (7, 3),         # Intermediate point
        #         (8, 3.5),       # Intermediate point
        #         (9, 4),         # Intermediate point
        #         (9, 5),         # Intermediate point
        #         (9, 6),         # Intermediate point
        #         (8, 6),         # Intermediate point
        #         (7, 7),         # Intermediate point
        #         (6, 7),         # Intermediate point
        #         (5, 7.5),       # Intermediate point
        #         (5, 8),         # Intermediate point
        #         (4, 8),         # Intermediate point
        #         (3, 8),         # Intermediate point
        #         (2, 8),         # Intermediate point
        #         (1, 8),         # Intermediate point
        #         (1, 7),         # Intermediate point
        #         (0, 7),         # Intermediate point
        #         (0, 6),         # Intermediate point
        #         (0, 5),         # Navigate towards an intersection or different area
        #         (1, 5),         # Intermediate point
        #         (2, 5),         # Intermediate point
        #         (2, 4),         # Intermediate point
        #         (3, 4),         # Intermediate point
        #         (3, 3),         # Intermediate point
        #         (4, 3),         # Intermediate point
        #         (4, 2),         # Intermediate point
        #         (3, 2),         # Intermediate point
        #         (2, 2),         # Intermediate point
        #         (1, 2),         # Intermediate point
        #         (0, 2),         # Intermediate point
        #         (0, 1),         # Intermediate point
        #         (1, 1),         # Intermediate point
        #         (2, 1),         # Intermediate point
        #         (3, 1),         # Move further into the room or corridor
        #         (4, 1),         # Intermediate point
        #         (5, 1),         # Intermediate point
        #         (5, 2),         # Navigate towards an intersection or different area
        #         (6, 2),         # Intermediate point
        #         (7, 2),         # Intermediate point
        #         (8, 2),         # Intermediate point
        #         (8, 3),         # Move further into the hospital area
        #         (7, 3),         # Intermediate point
        #         (6, 3),         # Intermediate point
        #         (5, 3),         # Intermediate point
        #         (5, 4),         # Intermediate point
        #         (4, 4),         # Intermediate point
        #         (3, 4),         # Intermediate point
        #         (3, 5),         # Intermediate point
        #         (2, 5),         # Intermediate point
        #         (1, 5),         # Intermediate point
        #         (1, 4),         # Intermediate point
        #         (0, 4),         # Intermediate point
        #         (0, 3),         # Intermediate point
        #         (1, 3),         # Intermediate point
        #         (2, 3),         # Intermediate point
        #         (2, 2),         # Intermediate point
        #         (3, 2),         # Intermediate point
        #         (4, 2),         # Intermediate point
        #         (5, 2),         # Intermediate point
        #         (6, 2),         # Intermediate point
        #         (6, 3),         # Intermediate point
        #         (7, 3),         # Intermediate point
        #         (8, 3),         # Intermediate point
        #         (9, 3),         # Intermediate point
        #         (9, 4),         # Intermediate point
        #         (9, 5),         # Intermediate point
        #         (8, 5),         # Intermediate point
        #         (7, 5),         # Intermediate point
        #         (6, 5),         # Intermediate point
        #         (5, 5),         # Intermediate point
        #         (4, 5),         # Intermediate point
        #         (3, 5),         # Intermediate point
        #         (2, 5),         # Intermediate point
        #         (1, 5),         # Navigate towards an intersection or different area
        #         (1, 6),         # Intermediate point
        #         (2, 6),         # Intermediate point
        #         (3, 6),         # Intermediate point
        #         (4, 6),         # Intermediate point
        #         (5, 6),         # Intermediate point
        #         (6, 6),         # Intermediate point
        #         (7, 6),         # Intermediate point
        #         (8, 6),         # Intermediate point
        #         (9, 6),         # Intermediate point
        #         (9, 7),         # Intermediate point
        #         (8, 7),         # Intermediate point
        #         (7, 7),         # Intermediate point
        #         (6, 7),         # Intermediate point
        #         (5, 7),         # Intermediate point
        #         (4, 7),         # Intermediate point
        #         (3, 7),         # Intermediate point
        #         (2, 7),         # Intermediate point
        #         (1, 7),         # Intermediate point
        #         (1, 6),         # Intermediate point
        #         (1, 5),         # Navigate towards an intersection or different area
        #         (1, 4),         # Intermediate point
        #         (1, 3),         # Intermediate point
        #         (1, 2),         # Intermediate point
        #         (2, 2),         # Intermediate point
        #         (2, 3),         # Intermediate point
        #         (3, 3),         # Intermediate point
        #         (3, 2),         # Intermediate point
        #         (4, 2),         # Intermediate point
        #         (5, 2),         # Intermediate point
        #         (5, 1),         # Intermediate point
        #         (5, 0),         # Intermediate point
        #         (5, 0)  ]        # End point (same as starting point)

                #   (0, 0),
                #         (0.125, 0.425),
                #         (0.25, 0.85),
                #         (0.375, 1.275),
                #         (0.5, 1.7),
                #         (0.625, 1.85),
                #         (0.75, 2),
                #         (0.875, 2.15),
                #         (1, 2.3),
                #         (1.125, 2.35),
                #         (1.25, 2.4),
                #         (1.375, 2.45),
                #         (1.5, 2.5),
                #         (1.625, 2.55),
                #         (1.75, 2.6),
                #         (1.875, 2.65),
                #         (2, 2.7),
                #         (2.125, 2.7),
                #         (2.25, 2.7),
                #         (2.375, 2.7),
                #         (2.5, 2.7),
                #         (2.625, 2.7),
                #         (2.75, 2.7),
                #         (2.875, 2.7),
                #         (3, 2.7),
                #         (3.125, 2.65),
                #         (3.25, 2.6),
                #         (3.375, 2.55),
                #         (3.5, 2.5),
                #         (3.625, 2.45),
                #         (3.75, 2.4),
                #         (3.875, 2.35),
                #         (4, 2.3),
                #         (4.125, 2.25),
                #         (4.25, 2.2),
                #         (4.375, 2.15),
                #         (4.5, 2.1),
                #         (4.625, 1.9),
                #         (4.75, 1.7),
                #         (4.875, 1.275),(4.9,1.175),(4.95,1),(5,0.75),(5,0.6),(5,0.5),(5,0.25),
                #         (5, 0)
                #         ]

        # Initialize plotting
        plt.ion()  
        self.fig, self.ax = plt.subplots()
        self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
        self.waypoints_plot, = self.ax.plot(self.waypoints_x, self.waypoints_y, 'b--', label='Waypoints')
        self.trajectory_plot, = self.ax.plot([], [], 'r-', label='Trajectory')
        self.robot_position_plot = self.ax.plot([], [], 'r^', markersize=10, label='Robot Position')[0]  
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

    # def tuple_list_callback(self, msg):
    #     self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
    #     self.x_goal_point = msg.a[0]
    #     self.y_goal_point = msg.b[0]
    #     self.waypoints.reverse()
    #     self.update_waypoints_plot()

    def map_velocity(self,velocity):
        # Clamping the value to be within the range -1.57 to 1.57
        velocity = min(max(velocity, -self.max_velocity), self.max_velocity)
        
        if velocity < 0:
            # Mapping negative values from -1.57 to 0 to the range 0 to 61
            return int(((velocity + self.max_velocity) / self.max_velocity) * 61)
        else:
            # Mapping positive values from 0 to 1.57 to the range 67 to 127
            return int((velocity / self.max_velocity) * 60 + 67)

    def purePursuit(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("Goal reached")
            self.plot_trajectory()
            return

        lookahead_point = self.find_lookahead_point()

        goal_x, goal_y = self.waypoints[-1]
        distance_to_goal = math.hypot(goal_x - self.currentx, goal_y - self.currenty)

        if distance_to_goal < 0.05:  # If the robot is close to the goal
            rospy.loginfo("Goal reached")
            Vr = 0.0
            Vl = 0.0
            self.vel_msg.x = Vr 
            self.vel_msg.y = Vl 

            self.vel_pub.publish(self.vel_msg)

            self.velocitylf_publisher.publish(Vr)
            self.velocityrf_publisher.publish(Vl)

            self.plot_trajectory()
            return

        angle_to_point = math.atan2(lookahead_point[1] - self.currenty, lookahead_point[0] - self.currentx)
        angle_diff = angle_to_point - self.robot_theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize the angle

        # If the waypoint is behind the robot, rotate in place
        if abs(angle_diff) > math.pi / 2:
            rospy.loginfo("Waypoint is behind the robot, rotating in place")
            if angle_diff > 0:
                Vr = 2.5
                Vl = -2.5

                self.vel_msg.x = self.map_velocity(Vr) 
                self.vel_msg.y = self.map_velocity(Vl) 

            else:
                Vr = -2.5
                Vl = 2.5

                self.vel_msg.x = self.map_velocity(Vr) 
                self.vel_msg.y = self.map_velocity(Vl) 


            self.vel_pub.publish(self.vel_msg)

            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)
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
        print("Vr: ", Vr, "Vl: ", Vl , "Mapped Vr: ", self.vel_msg.x ,"Mapped Vl: ", self.vel_msg.y )

        self.plot_trajectory()

    def find_lookahead_point(self):
        lookahead_point = None
        ld = self.min_ld
        
        while ld <= self.max_ld:
            for i in range(self.current_waypoint_index, len(self.waypoints)):
                point = self.waypoints[i]
                distance = math.hypot(point[0] - self.currentx, point[1] - self.currenty)
                if distance >= ld:
                    lookahead_point = point
                    self.current_waypoint_index = i
                    rospy.loginfo(f"Selected lookahead point {lookahead_point} at distance {distance}, ld={ld}")
                    break
            if lookahead_point:
                break
            ld += 0.1  # Increment ld only if a lookahead point hasn't been found

        if not lookahead_point:
            lookahead_point = self.waypoints[-1]
            rospy.loginfo(f"Using final goal at {self.waypoints[-1]} as lookahead point")
        
        return lookahead_point

    def plot_trajectory(self):
        self.trajectory_x, self.trajectory_y = zip(*self.robot_trajectory)
        self.trajectory_plot.set_data(self.trajectory_x, self.trajectory_y)
        self.robot_position_plot.set_data([self.currentx], [self.currenty]) 
        self.fig.canvas.flush_events()
        plt.pause(0.001)

if __name__ == '__main__':
    x = Control()
    while not rospy.is_shutdown():
        x.purePursuit()
        rospy.sleep(0.1)
