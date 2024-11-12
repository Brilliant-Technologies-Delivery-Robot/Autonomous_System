#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdometryVisualizer:
    def __init__(self):
        rospy.init_node('odometry_visualizer', anonymous=True)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.amcl_x = []
        self.amcl_y = []
        self.gazebo_x = []
        self.gazebo_y = []

    def amcl_callback(self, data):
        self.amcl_x.append(data.pose.pose.position.x)
        self.amcl_y.append(data.pose.pose.position.y)

    def gazebo_callback(self, data):   
        self.gazebo_x.append(data.pose[6].position.x)
        self.gazebo_y.append(data.pose[6].position.y)

    def plot(self):
        plt.figure()
        plt.plot(self.amcl_x, self.amcl_y, label='AMCL Odometry')
        plt.plot(self.gazebo_x, self.gazebo_y, label='Gazebo Ground Truth')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Odometry Visualization')
        plt.legend()
        plt.show()

if __name__ == "__main__":
    visualizer = OdometryVisualizer()
    rospy.spin()
    visualizer.plot()
