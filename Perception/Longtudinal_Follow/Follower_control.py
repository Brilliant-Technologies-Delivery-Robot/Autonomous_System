#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import threading
import time

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

class Lontudinal_Control:
    def __init__(self):
        rospy.init_node('position_control', anonymous=True)

        # Initial PID controller parameters
        self.kp_z = 6.0 # Lowered to reduce overshoot
        self.ki_z = 0.0  # Slightly increased to eliminate steady-state error
        self.kd_z = 0.0  # Increased to reduce oscillations

        self.pid_controller_z = PIDController(self.kp_z, self.ki_z, self.kd_z)

        self.right_wheel_pub = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.error_pub = rospy.Publisher('/distance_error', Float64, queue_size=10)

        self.current_distance = 0.0
        self.desired_distance = 2.2
        self.base_speed = 0.25

        self.distance_sub = rospy.Subscriber('depth_center_point', Point, self.distance_callback)

        # To store errors for plotting
        self.errors = []
        self.times = []
        self.start_time = time.time()

        # Start a separate thread for plotting
        self.plot_thread = threading.Thread(target=self.plot_error)
        self.plot_thread.start()

    def distance_callback(self, data):
        self.current_distance = data.z
        self.update_control()

    def update_control(self):
        error_distance = self.current_distance - self.desired_distance
        control_z = self.pid_controller_z.compute(error_distance)

        # Adjust wheel speeds based on the control signal
        right_wheel_speed = -(self.base_speed + control_z)
        left_wheel_speed = -(self.base_speed + control_z)

        rospy.loginfo(f"Current Distance: {self.current_distance}")
        rospy.loginfo(f"Desired Distance: {self.desired_distance}")
        rospy.loginfo(f"Error in Distance: {error_distance}")
        rospy.loginfo(f"Control Z: {control_z}")
        rospy.loginfo(f"Right Wheel Speed: {right_wheel_speed}, Left Wheel Speed: {left_wheel_speed}")

        self.right_wheel_pub.publish(right_wheel_speed)
        self.left_wheel_pub.publish(left_wheel_speed)
        self.error_pub.publish(error_distance)  # Publish the error

        # Append the error and the current time to lists
        current_time = time.time() - self.start_time
        self.errors.append(error_distance)
        self.times.append(current_time)

    def plot_error(self):
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()
        line, = ax.plot(self.times, self.errors, 'r-')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.set_title('Distance Error Over Time')

        while not rospy.is_shutdown():
            if self.errors:  # Update plot only if there are errors recorded
                line.set_xdata(self.times)
                line.set_ydata(self.errors)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()
            time.sleep(0.1)  # Update the plot every 100 ms

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        position_control = Lontudinal_Control()
        position_control.spin()
    except rospy.ROSInterruptException:
        pass
