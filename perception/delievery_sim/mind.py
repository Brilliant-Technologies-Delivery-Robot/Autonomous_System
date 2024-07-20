#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32

class CombinedListener:
    def __init__(self):
        rospy.init_node('combined_listener', anonymous=True)
        rospy.Subscriber("/detected_object_label", String, self.callback_label)
        rospy.Subscriber("/ultrasonic/distance", Float32, self.callback_distance)
        self.previous_distance = 0
        self.state_msg = String
        self.Threshold = 0.1
        rospy.spin()

    def callback_label(self, data):
        self.label = data.data
        self.log_combined_message()

    def callback_distance(self, data):
        self.distance = data.data
        self.log_combined_message()
        current_distance = self.distance
        if self.previous_distance is not None:
            distance_difference = abs(current_distance - self.previous_distance)

            if distance_difference < self.Threshold:
                self.state_msg =  "moving towards"
                return self.state_msg
            elif current_distance < self.previous_distance:
                self.state_msg =  "moving Away"
                return self.state_msg
            else:
                self.state_msg =  "Stationary"
                return self.state_msg       
                

    def log_combined_message(self):
        if hasattr(self, 'label') and hasattr(self, 'distance'):
            rospy.loginfo("Received : %s at distance: %f and object is: ", self.label, self.distance)
            rospy.loginfo(self.state_msg)
            
if __name__ == '__main__':
    CombinedListener()
