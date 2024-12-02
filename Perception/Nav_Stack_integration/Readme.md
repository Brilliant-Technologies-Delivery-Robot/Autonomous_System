# Navigation Goal Sender using Kinect Camera

This project consists of two Files that interface with the ROS navigation stack to send goals for a mobile robot. Each Files uses data from a Kinect camera to determine the target goals dynamically. But one deals with the robot in the simulation and another one the real-life


## Prerequisites

Ensure you have the following installed and set up:

1. ROS (Robot Operating System) - Tested with ROS Noetic/ROS 2 if applicable.
2. Kinect Camera Drivers and SDK.
3. A configured ROS navigation stack.
4. Required dependencies:
   - `rospy` for ROS Python nodes.
   - `sensor_msgs` for Kinect camera data.
   - `geometry_msgs` for sending navigation goals.

## Structure

Nav_Stack_integration
&nbsp;&nbsp;&nbsp;&nbsp;|
&nbsp;&nbsp;&nbsp;&nbsp;|_______ codes_in_sim
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ conversion_transform.py
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ kinect_sub.py
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ stop_at_goal.py
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ sub_convert_to_meter.py
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ sub_depth_point.py
&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ sub_send_goal.py
&nbsp;&nbsp;&nbsp;&nbsp;|
&nbsp;&nbsp;&nbsp;&nbsp;|_______ codes_in_real_life
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ package.xml
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ CMakeLists.txt
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ convert_error_to_meter.py
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ kinect_frame_pub.py
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ kinect_frame_sub_detect.py
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|_____ sub_detgepthimage.py

The python files of in the both folders are similar. only the main difference that the codes in real life is a package file and to run the launch file of these codes you have first run the following.

```bash
 roslaunch freenect_launch freenect.launch depth_registration:=true
```

## Deploy Real-Life Codes
To use these files and the launch file attached in the folder. 
```bash
    git clone -b <branch_name> <repo_url>
    catkin build
    roslaunch <package_name> real_life_kinect.launch
```

## Codes Explanation
- codes_in_real_life --> kinect_frame_pub.py:
 This python file mainly used to publish the frame that the kinect can see. and there'e no similar file in the codes_in_sim folder

- codes_in_real_life --> kinect_frame_sub_detect.py && codes_in_sim --> kinect_sub.py:
both of these python files subs the frame publish even if it published from another python code or gazebo and after it receives it. it applies detection model to it,

- codes_in_real_life --> sub_detgepthimage.py && codes_in_sim --> sub_depth_point.py:
After getting the frame these python files gets the center point of the detected bounding box. and gets the depth of the centeer of the object to make it easier to get x,y,z of the object in the space.

- codes_in_real_life --> convert_error_to_meter.py && codes_in_sim --> sub_convert_to_meter.py:
We use the calibration data of the camera to convert these data from the pixel domain into real life dimensions in meters.

- codes_in_sim --> conversion_transform.py:
This file used in simulation to make transformation from the camera frame to the base link.