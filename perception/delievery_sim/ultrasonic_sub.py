#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32  

class UltrasonicNode:
    def __init__(self):

        rospy.init_node('ultrasonic_node', anonymous=True)
        self.distance_publisher = rospy.Publisher("/ultrasonic/distance", Float32, queue_size=10)
        rospy.Subscriber("/sensor/ultrasonic", Range, self.ultrasonic_callback)
        rospy.spin()

    def ultrasonic_callback(self, data):
        distance = data.range
        rospy.loginfo("Ultrasonic distance: %f meters", distance)  
        self.distance_publisher.publish(distance) # Publish the distance


if __name__ == '__main__':
    UltrasonicNode()
