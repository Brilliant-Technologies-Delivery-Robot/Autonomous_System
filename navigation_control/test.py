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

        # self.path_subscriber = rosp45185y.Subscriber('tuple_list_topic', wp_list, self.tuple_list_callback)  

        self.pose = ModelStates()
        self.throttle_output=Float64()

        self.rate = rospy.Rate(10)
        self.kp = 0.1
        self.ki = 1
        self.kd = 0.0
        self.dist_ld =0.6

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.max_velocity = 5

        self.robot_theta = 0.0
        self.width = 0.5
        self.Vl=10
        self.Vr=10
    
    def update_pose(self, data:ModelStates):
        self.pose = data
        self.currentx = self.pose.pose[1].position.x 
        self.currenty = self.pose.pose[1].position.y
        orientation = self.pose.pose[1].orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def test(self):
        self.velocitylf_publisher.publish(self.Vl)
        self.velocityrf_publisher.publish(self.Vr)
        print ("Vl: ", self.Vl, "Vr: ", self.Vr)


if __name__ == '__main__':
    try:
        x = Control()
        while not rospy.is_shutdown():
            x.test()
    except rospy.ROSInterruptException:
        pass
