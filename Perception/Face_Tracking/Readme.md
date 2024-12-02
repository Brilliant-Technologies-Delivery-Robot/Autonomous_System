# Face-Tracking Robot with YOLO and PID Control

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Project Workflow](#project-workflow)
- [Architecture](#architecture)
- [Implementation](#implementation)
  - [ROS Nodes](#ros-nodes)
  - [Launch File](#launch-file)
- [Results](#results)


## Overview
This project implements a face-tracking robot simulation in Gazebo, as well as a real-life counterpart. The robot detects a face using YOLO, calculates the lateral error, and uses a PID controller to adjust its motion to minimize the error. This ensures the robot can effectively follow a target while maintaining alignment.

## Prerequisites
Before running the project, ensure you have the following installed:

1. **ROS (Robot Operating System)**  
   Install ROS by following the official [ROS installation guide](http://wiki.ros.org/ROS/Installation).

2. **Gazebo**  
   Install Gazebo for the simulation. Check [Gazebo's official website](http://gazebosim.org/) for installation instructions.

3. **Python**  
   Install Python (3.x recommended) and the following libraries:
   ```bash
   pip install rospy opencv-python numpy ultralytics cvzone matplotlib
   ```
### Adding Mono camera to the original urdf
```bash

    <?xml version="1.0" encoding="utf-8"?>

    <!-- Mono Camera -->
    <joint name="mono_camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="mono_camera_link"/>
    <origin xyz="-0.305 0 0.8" rpy="0 0 3.14"/>
    </joint>   
    <link name="mono_camera_link">
    <visual>
        <geometry>
            <box size="0.010 0.03 0.03"/>
        </geometry>
        <material name="Gazebo/Red"/> <!-- Using predefined Gazebo material -->
    </visual>
    </link>
    <joint name="mono_camera_optical_joint" type="fixed">
    <parent link="mono_camera_link"/>
    <child link="mono_camera_link_optical"/>
    <origin xyz="0 0 0" rpy="-1.57 0 -1.57"/>
    </joint>
    <link name="mono_camera_link_optical"></link>
    <gazebo reference="mono_camera_link">
    <material>Gazebo/Red</material>
    <sensor name="mono_camera" type="camera">
        <pose> 0 0 0 0 0 0 </pose>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <camera>
            <horizontal_fov>1.089</horizontal_fov>
            <image>
                <format>R8G8B8</format>
                <width>640</width>
                <height>480</height>
            </image>
            <clip>
                <near>0.05</near>
                <far>8.0</far>
            </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <frame_name>mono_camera_link_optical</frame_name>
            <camera_name>mono_camera</camera_name> <!-- Name of the camera -->
            <image_topic_name>/mono_camera/image_raw</image_topic_name> <!-- Topic for raw image data -->
            <camera_info_topic_name>/mono_camera/camera_info</camera_info_topic_name> <!-- Topic for camera info -->
        </plugin>
    </sensor>
    </gazebo>
```

## Project Workflow
The project is structured into three Python nodes, managed through a ROS launch file:

1. **Camera Frame Publisher**:  
   Publishes real-time frames from the camera in both simulation (Gazebo) and real-world scenarios.
   
2. **YOLO Face Detection and Lateral Error Calculation**:  
   - Subscribes to the camera frame.  
   - Applies YOLO-based face detection to identify the target's position.  
   - Calculates the lateral error (the horizontal offset between the detected face and the center of the camera frame).  
   - Publishes the calculated lateral error as a ROS topic.  

3. **PID Controller Node**:  
   - Subscribes to the lateral error topic.  
   - Uses a PID controller to minimize the lateral error by controlling the robot's motion.  
   - Publishes velocity commands to move the robot in Gazebo or a real-world environment.

## Architecture
The following diagram outlines the system architecture:
Camera --> [Frame Publisher] --> [YOLO Node] --> [Lateral Error Topic] --> [PID Controller] --> Robot Motion. The same python files are applied even in the simulation or in the real life.


## Implementation
### ROS Nodes
1. **Frame Publisher**:
   - Launches the camera in simulation or real life.
   - Publishes frames on the `/mono_camera/image_processed` topic in the simulation but in the real life the topic name is `/camera_frame`.

2. **YOLO Node**:
   - Subscribes to `/camera_frame` in real life or `/mono_camera/image_processed` in sim.
   - Processes the frame using YOLO for face detection.
   - Calculates the lateral error based on the face's position relative to the camera's center.
   - Publishes the lateral error to `/face_real_error_x`.

3. **PID Controller Node**:
   - Subscribes to `//face_real_error_x`.
   - Adjusts the robot's motion to reduce the lateral error using a PID control loop.
   - Publishes velocity commands to the `/cmd_vel` topic.

### Launch File
To run all the python files after making it in a package file in your workspace , just make the following.
   ```bash
    roslaunch <package_name> <launch_file_name>
   ```

## Results

- Images
<div style="display: flex; justify-content: space-between;">
  <div style="text-align: center;">
    <img src="/Perception/Face_Tracking/Results/Frame.png" alt="Simulation Example" width="90%">
    <p><strong>Figure 1:</strong> Simulation Example</p>
  </div>
  <div style="text-align: center;">
    <img src="/Perception/Face_Tracking/Results/output_image.jpg" alt="Real-Life Example" width="90%">
    <p><strong>Figure 2:</strong> Real-Life Example</p>
  </div>
</div>

- GIF
<div style="text-align: center;">
  <img src="/Perception/Face_Tracking/Results/InShot_20240908_021205687-ezgif.com-video-to-gif-converter.gif" alt="Robot Tracking in Action" width="100%">
  <p><strong>Figure 3:</strong> Robot Tracking in Action</p>
</div>