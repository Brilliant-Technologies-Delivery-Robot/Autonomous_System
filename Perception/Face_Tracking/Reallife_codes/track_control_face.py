#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Pose,Vector3

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class PositionControl:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('position_control', anonymous=True)

        # PID controller parameters
        self.kp = 0.003  # Adjust as needed
        self.ki = 0.0
        self.kd = 0.0
        self.pid_controller = PIDController(self.kp, self.ki, self.kd)
        self.vel_pub = rospy.Publisher('vel', Vector3, queue_size=10)
        
        # Initialize wheel command publishers
        self.right_wheel_pub = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)

        # Initialize error variable
        self.error_x = 0.0
        self.base_speed = 0.004  # Base speed for forward movement
        self.max_velocity = 10
        self.vel_msg = Vector3()

        # Subscribe to the X-axis error topic
        self.error_x_sub = rospy.Subscriber('/error_x', Int16, self.error_x_callback)

    def error_x_callback(self, data):
        self.error_x = data.data
        self.update_control()

    def update_control(self):
        # Compute control output
        control_x = self.pid_controller.compute(self.error_x)

        # Determine the speed commands for the wheels
        if abs(self.error_x) <= 5:
            # Error is within the deadzone, stop the robot
            right_wheel_speed = 0
            left_wheel_speed = 0
        elif self.error_x > 0:
            # Error is positive, rotate left
            right_wheel_speed = self.base_speed - control_x
            left_wheel_speed = 0
        else:
            # Error is negative, rotate right
            right_wheel_speed = 0
            left_wheel_speed = self.base_speed + control_x

        # Publish control commands
        self.right_wheel_pub.publish(right_wheel_speed)
        self.left_wheel_pub.publish(left_wheel_speed)

        self.vel_msg.x = self.map_velocity(right_wheel_speed) 
        self.vel_msg.y = self.map_velocity(left_wheel_speed) 
        self.vel_pub.publish(self.vel_msg)

        # Print the error and wheel speeds
        rospy.loginfo(f"Error in X: {self.error_x}, Right Wheel Speed: {right_wheel_speed}, Left Wheel Speed: {left_wheel_speed}")
    
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
    
    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        position_control = PositionControl()
        position_control.spin()
    except rospy.ROSInterruptException:
        pass
