#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int16
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

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

        # Initialize wheel command publishers
        self.right_wheel_pub = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)

        # Initialize error variable
        self.error_x = 0.0
        self.base_speed = 0.004  # Base speed for forward movement

        # Initialize Matplotlib plot
        self.fig, self.ax = plt.subplots()
        self.error_data = []
        self.time_data = []
        self.start_time = rospy.get_time()
        self.plot_initialized = False

        # Subscribe to the X-axis error topic
        self.error_x_sub = rospy.Subscriber('/face_error_x', Int16, self.error_x_callback)

        # Start a separate thread for plotting
        self.plot_thread = threading.Thread(target=self.plot_errors)
        self.plot_thread.start()

    def error_x_callback(self, data):
        self.error_x = data.data
        self.update_control()

        # Update plot data
        current_time = rospy.get_time() - self.start_time
        self.time_data.append(current_time)
        self.error_data.append(self.error_x)

        if len(self.time_data) > 100:
            self.time_data.pop(0)
            self.error_data.pop(0)

    def update_control(self):
        # Compute control output
        control_x = self.pid_controller.compute(self.error_x)

        # Determine the speed commands for the wheels
        if abs(self.error_x) <= 5:
            # Error is within the deadzone, stop the robot
            right_wheel_speed = self.base_speed
            left_wheel_speed = self.base_speed
        elif self.error_x > 0:
            # Error is positive, rotate left
            right_wheel_speed = 0
            left_wheel_speed = self.base_speed - control_x
        else:
            # Error is negative, rotate right
            right_wheel_speed = self.base_speed + control_x
            left_wheel_speed = 0

        # Publish control commands
        self.right_wheel_pub.publish(right_wheel_speed)
        self.left_wheel_pub.publish(left_wheel_speed)

        # Print the error and wheel speeds
        rospy.loginfo(f"Error in X: {self.error_x}, Right Wheel Speed: {right_wheel_speed}, Left Wheel Speed: {left_wheel_speed}")

    def plot_errors(self):
        # Update plot in real-time
        while not rospy.is_shutdown():
            if self.plot_initialized:
                self.ax.clear()
            self.ax.plot(self.time_data, self.error_data, label='Error X')
            self.ax.set_xlabel('Time (s)')
            self.ax.set_ylabel('Error X')
            self.ax.set_title('Error X Over Time')
            self.ax.legend()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            rospy.sleep(0.1)  # Adjust plot update frequency here

    def spin(self):
        # Keep the node running
        rospy.spin()
        # Close the plot window on shutdown
        plt.close(self.fig)

if __name__ == '__main__':
    try:
        position_control = PositionControl()
        position_control.spin()
    except rospy.ROSInterruptException:
        pass
