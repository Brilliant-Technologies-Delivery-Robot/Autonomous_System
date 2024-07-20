# Delivery Robot with Kinect Camera Simulation

This project involves the simulation of a delivery robot equipped with a Kinect camera. The simulation is implemented using ROS and Gazebo, and the camera is configured to provide RGB and depth images along with point cloud data.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [URDF and Gazebo Configuration](#urdf-and-gazebo-configuration)
- [Topics](#topics)

## Introduction
This project simulates a delivery robot with a Kinect camera using ROS and Gazebo. The camera is configured to provide RGB images, depth images, and point cloud data, which can be used for various perception tasks.

## Prerequisites
- ROS Noetic
- Gazebo 11
- catkin build tools

Ensure you have ROS and Gazebo installed on your system. Follow the instructions on the ROS [installation page](http://wiki.ros.org/noetic/Installation) and Gazebo [installation page](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

## Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/delivery_robot.git
    cd delivery_robot
    ```

2. Build the workspace:
    ```bash
    catkin_make
    source devel/setup.bash
    ```

3. Add the package path to your ROS environment:
    ```bash
    echo "source /path/to/your/workspace/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## Usage
To launch the simulation with the Kinect camera:
```bash
roslaunch delivery_robot delivery_robot_gazebo.launch
```
## URDF and Gazebo Configuration
- URDF Configuration

The URDF file defines the physical and visual properties of the Kinect camera mounted on the delivery robot. and we have two kinect frames the optical one is to transfrom the frame in a right way.Below is a snippet of the URDF configuration:

```bash
<!-- Camera Gazebo Model-->
<link name="kinect_body">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
          <mesh filename="package://delivery_robot/kinect/meshes/kinect.dae" />
      </geometry>
  </visual>
  <collision>
      <geometry>
          <box size="0.07 0.3 0.09"/>
      </geometry>
  </collision>
</link>

<joint name="camera_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  <parent link="kinect_body"/>
  <child link="camera_frame_optical"/>
</joint>
<link name="camera_frame_optical"/>

<joint name="camera_frame_joint" type="fixed">
  <origin xyz="-0.25 0 0.4" rpy="0 0 3.14"/>
  <parent link="base_link"/>
  <child link="kinect_body"/>
</joint>

```
## Gazebo Configuration

The Gazebo configuration defines the camera sensor and its parameters, including the topics for RGB images, depth images, and point cloud data.

```bash
<gazebo reference="kinect_body">
  <sensor type="depth" name="camera">
    <always_on>true</always_on>
    <update_rate>20.0</update_rate>
    <camera>
      <horizontal_fov>60</horizontal_fov>
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
    <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <cameraName>camera</cameraName>
      <alwaysOn>true</alwaysOn>
      <updateRate>10</updateRate>
      <imageTopicName>/rgb/image_raw</imageTopicName>
      <depthImageTopicName>/depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>/depth/points</pointCloudTopicName>
      <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
      <frameName>camera_frame_optical</frameName>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <pointCloudCutoff>0.4</pointCloudCutoff>
    </plugin>
  </sensor>
</gazebo>

```
## Topics
The Kinect camera publishes the following topics:

    /rgb/image_raw: RGB image data
    /depth/image_raw: Depth image data
    /depth/points: Point cloud data
    /rgb/camera_info: Camera intrinsic parameters
    /depth/camera_info: Depth camera intrinsic parameters