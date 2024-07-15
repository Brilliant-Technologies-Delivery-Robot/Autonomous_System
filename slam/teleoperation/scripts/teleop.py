#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8MultiArray, Float64
from os import system
from tkinter import Tk, Event

class ControlApp(Tk, object):

    def __init__(self) -> None:
        super(ControlApp, self).__init__()
        self.init_node()
        self.config()
        self.init_keyboard()
        self.mainloop()

    def init_node(self) -> None:
        rospy.init_node("teleop_node")
        self.velocitylf_publisher = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.velocityrf_publisher = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)
        
        
    def init_keyboard(self) -> None:
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)

    def config(self) -> None:
        self.forw = 1.57
        self.stop = 0
        self.back = -1.57
        self.speeds_msg = Int8MultiArray()

    def keydown(self, event: Event) -> None:
        if event.keysym == "Up":
            self.velocitylf_publisher.publish(self.forw)
            self.velocityrf_publisher.publish(self.forw)
            
        elif event.keysym == "Down":
            self.velocitylf_publisher.publish(self.back)
            self.velocityrf_publisher.publish(self.back)
            
        elif event.keysym == "Left":
            self.velocitylf_publisher.publish(self.forw)
            self.velocityrf_publisher.publish(self.back)
            
        elif event.keysym == "Right":
            self.velocitylf_publisher.publish(self.back)
            self.velocityrf_publisher.publish(self.forw)
            

    def keyup(self, event: Event) -> None:
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            self.velocitylf_publisher.publish(self.stop)
            self.velocityrf_publisher.publish(self.stop)
            

if __name__ == "__main__":
    try:
        system('xset r off')
        control = ControlApp()
        system('xset r on')
    except rospy.ROSInterruptException:
        system('xset r on')
        
