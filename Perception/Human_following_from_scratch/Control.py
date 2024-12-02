#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Vector3,Point
from nav_msgs.msg import Odometry,Path
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
        self.error_xy_meter_sub = rospy.Subscriber('/transformed_cord', Point, self.xy_callback)

        self.vel_pub = rospy.Publisher('vel', Vector3, queue_size=10)   
        
        self.pose = ModelStates()
        self.throttle_output = Float64()
        self.vel_msg = Vector3()

        self.rate = rospy.Rate(10)
        self.kp = 20
        self.ki = 1.2
        self.kd = 0.1
        self.dist_ld = 0.25
        self.min_ld = 1# Minimum lookahead distance
        self.max_ld = 3 # Maximum lookahead distance

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.current_waypoint_index = 0
        self.desired_goal = (0.0,0.0)  # Set to default point if no valid data

        self.max_velocity = 4

        self.robot_theta = 0.0
        self.width = 0.45
        self.time_values = []
        self.error_values = []
        self.robot_trajectory = []  
        self.waypoints= []
        
    def update_pose(self, data):
        self.pose = data
        self.currentx = data.pose[1].position.x
        self.currenty = data.pose[1].position.y
        orientation_q = data.pose[1].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.robot_theta = self.yaw
        self.robot_trajectory.append((self.currentx, self.currenty))

    def xy_callback(self, data):
        # If no valid goal point is detected, stop the robot
        # print("Yaw Angle",self.yaw)
        x = self.currentx + data.x * math.cos(self.yaw) + data.y * math.sin(self.yaw)
        y = self.currenty + data.x * math.sin(self.yaw) - data.y * math.cos(self.yaw)

        # print("Current Positions", self.currentx , self.currenty)
        if data.x == 0 and data.z == 0:
            rospy.loginfo("No human detected. Stopping the robot.")
            
        else:
            # Set goal point with updated values
            self.desired_goal = (x , y)  # Assuming z=0 as it's not used
            # rospy.loginfo(f"Subscribed to /error_xy_meter: Desired Goal = ({self.desired_goal})")
            self.waypoints.append(self.desired_goal)        

    def purePursuit(self):
      
        lookahead_point = list(self.desired_goal)
        e = math.hypot(lookahead_point[0] - self.currentx , lookahead_point[1] - self.currenty)
        
        # print(e)
        # Check if the robot is close to the goal
        if e < 1.8:
            rospy.loginfo("Goal reached")
            Vr = Vl =  0.0      
            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)

            # rospy.loginfo(f"Vr: {Vr}, Vl: {Vl}, Mapped Vr: {self.vel_msg.x}, Mapped Vl: {self.vel_msg.y}")
            return

        angle_to_point = math.atan2(lookahead_point[1] - self.currenty, lookahead_point[0] - self.currentx)
        angle_diff = angle_to_point - self.robot_theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize the angle

        curvature = 2 * math.sin(angle_diff) / self.dist_ld

        # PID control for velocity
        error = e
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        velocity = min(velocity, self.max_velocity)

        Vr = velocity * (2 + curvature * self.width) / 2
        Vl = velocity * (2 - curvature * self.width) / 2

        Vr = min(max(Vr, -self.max_velocity), self.max_velocity)
        Vl = min(max(Vl, -self.max_velocity), self.max_velocity)

        print("Vr: ", Vr, "Vl: ", Vl)

        self.velocitylf_publisher.publish(Vl)
        self.velocityrf_publisher.publish(Vr)

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
                    rospy.loginfo(f"Selected lookahead point {lookahead_point} at distance {distance}")
                    break
            if lookahead_point:
                break
            ld += 0.05  # Increment ld only if a lookahead point hasn't been found
        return lookahead_point

if __name__ == '__main__':
    x = Control()
    while not rospy.is_shutdown():
        x.purePursuit()
        rospy.sleep(0.1)
