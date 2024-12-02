# Human Follow with the feedback of depth camera

## Overview
The pipeline made for detecting objects, processing depth images, and implementing a control algorithm for a robot to follow detected objects.

## Files Structure

1. **camera_stream_sim.py**

This script handles the streaming of RGB images from the Kinect camera in the Gazebo simulator. It subscribes to the "/rgb/image_raw" topic and republishes the processed frames to "/published/kinect/image_raw".

2. **camera_sub_sim.py**

This script performs object detection on the published RGB frames from "/published/kinect/image_raw". Using OpenCV's DNN module, it leverages a pre-trained SSD MobileNet model to detect objects, publish detected labels, and calculate the center of bounding boxes.

3. **sub_detdepthimage.py**
This script processes depth images from the "/depth/image_raw" topic and aligns them with the bounding box centers received from the "bounding_box_center" topic. It calculates errors in position (x, y) and depth (z) relative to a desired point (320, 240 pixels), representing the center of the camera frame.

4. **Follower_control.py**

This script uses the PID controller to monitor a fixed distance between the object and the robot using the depth info from the topic "depth_center_point".

## Results

<div style="text-align: center;">
  <img src="/Perception/Longtudinal_Follow/Result/clideo_editor_20a2f323a4a64e96a525253bf33f360c.gif" alt="Robot Following in Action" width="100%">
  <p><strong>Figure 1:</strong> Robot Following in Action</p>
</div>
