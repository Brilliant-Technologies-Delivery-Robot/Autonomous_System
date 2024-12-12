# Depth Estimation and Object Tracking Improvement

## Problem Overview

Initially, the depth estimation was inaccurate due to two primary issues:
1. Relying solely on the depth of the center point of the bounding box.
2. Bounding box jittering, which caused inconsistencies in tracking and depth estimation.

## Steps Taken to Solve the Problem

### 1. Addressing Inaccurate Depth
To improve depth estimation, I attempted to calculate the mean depth over the entire bounding box of the detected object. Three different methods were tested:
- **Exponential Moving Average (EMA)**
- **Simple Moving Average (SMA)**
- **Median Filter**

After evaluating these methods on the CPU, the **Median Filter** was found to be the most robust and reliable for averaging the depth values.

### 2. Solving Bounding Box Jittering
Even after applying the Median Filter, bounding box jittering still persisted, which impacted depth accuracy. To resolve this, I explored object tracking algorithms:
- Tested **ByteTracker** and **DeepSORT**.
  - **ByteTracker** resulted in an FPS between 15 to 20.
  - **DeepSORT** provided a better FPS range of 18 to 24, making it the preferred option for stable tracking and improved performance.

### 3. Improvement Using Instance Segmentation
The final enhancement involved **Instance Segmentation**. Although I had previously attempted this technique with limited success, it worked effectively when applied over the segmented object. I then used the **Median Filter** on the segmented parts, leading to a notable improvement in the depth accuracy and stability.

## Summary of Key Improvements

1. **Median Filter** applied to the depth values to reduce inaccuracies.
2. Integration of **DeepSORT Tracker** to address bounding box jittering and enhance tracking stability.
3. Successful implementation of **Instance Segmentation** to refine depth calculations and reduce jittering.

These three major changes significantly improved the depth estimation accuracy and the stability of object tracking.

## Files Structure
1. **real_life_kinect_det_track.launch** and **camera_send_goal_with_tracker.launch**
Both of these files used to launch the detection , tracking and sending goal files. the send goal one is the one that sends the actuation to the nav stack. the another one is only the detection pipeline

2. **kinect_seg_track.launch** and **seg_camera_send_goal.launch**
These Files are the same like the above but here we have changed in the camera pipeline and used segmentation which is more accurate but more computation required too 