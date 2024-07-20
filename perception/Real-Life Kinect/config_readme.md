# Kinect Camera Configuration on Ubuntu 20.04 with ROS Noetic

This guide provides step-by-step instructions to set up and configure a Kinect camera on Ubuntu 20.04 using ROS Noetic.


## Prerequisites

1. **Ubuntu 20.04** - Make sure you have Ubuntu 20.04 installed.
2. **ROS Noetic** - Ensure ROS Noetic is installed and properly set up. You can follow the official ROS installation guide: http://wiki.ros.org/noetic/Installation/Ubuntu

## Step-by-Step Guide

we have to download the two files {libfreenect - freenect_stack}.
- libfreenect  
we should download it generally and locate it any where. you could download it from this repo or follow the steps in the below link.
    ```sh
    https://aibegins.net/2020/11/22/give-your-next-robot-3d-vision-kinect-v1-with-ros-noetic/
    ```
- freenect_stack \
After we have downloaded the above file and followed the link we make workspace and do the following:
    ```sh
        mkdir catkin_ws
        cd catkin_ws 
        mkdir src
        cd src
        git clone https://github.com/ros-drivers/freenect_stack.git
        cd ..
        catkin_make
        source devel/setup.sh
        roslaunch freenect_launch freenect.launch depth_registration:=true
        rviz
    ```
In the same workspace i have developed a package that contains a code that shows all of the forms could be gotted from the kinect
- RGB Image 
- IR image
- Depth Image
- Point Cloud Data
