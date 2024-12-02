# Robot Human Following 
## Overview
This project involves implementing a system for human detection and control of a differential robot in a simulated Gazebo environment. The system is divided into two primary components:

    Human Detection: Detecting humans in the Gazebo world using an SSD (Single Shot Detector) model.
    Robot Control: Using the detected human's position to compute longitudinal and lateral errors, corrected via PID and Pure Pursuit controllers.

The pipeline is also extended for real-life applications with Kinect integration for frame publication and detection.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Detector](#detector)
   - [Model Description](#model-description)
   - [Model Loading](#model-loading)
   - [Model Results](#model-results)
4. [Controllers](#controllers)
   - [Longitudinal Controller (PID)](#longitudinal-controller-pid)
   - [Lateral Controller (Pure Pursuit)](#lateral-controller-pure-pursuit)
5. [Pipeline Explanation](#pipeline-explanation)
   - [Codes Structure](#codes-structure)
6. [Results](#results)

---

## Detector

### Model Description
The project uses a pre-trained SSD MobileNet V3 model to detect humans. The SSD detector includes:
- **Frozen Inference Graph**: Contains the trained model weights.
- **Configuration File**: Describes the model's architecture and input requirements.
- **Labels File**: Defines the classes the model can detect.

### Model Loading
The detector is loaded using OpenCV's DNN module as follows:

```python
import cv2

class HumanDetector:
    def __init__(self):
        self.config_file = 'path_to/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        self.frozen_model = 'path_to/frozen_inference_graph.pb'
        self.file_name = 'path_to/labels.txt'
        
        # Load the model
        self.model = cv2.dnn_DetectionModel(self.frozen_model, self.config_file)
        self.model.setInputSize(320, 320)
        self.model.setInputScale(1.0 / 127.5)
        self.model.setInputMean((127.5, 127.5, 127.5))
        self.model.setInputSwapRB(True)

    def detect(self, frame):
        classes, confidences, boxes = self.model.detect(frame, confThreshold=0.5)
        return classes, confidences, boxes
```

### Model Results

<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="/Perception/Human_following_from_scratch/Results/TrafficIPCameravideo-ezgif.com-video-to-gif-converter.gif" alt="First GIF" style="width: 45%;"/>
  <img src="/Perception/Human_following_from_scratch/Results/video.gif" alt="Second GIF" style="width: 45%;"/>
</div>

## Controllers

### Longitudinal Controller (PID)
The longitudinal controller ensures that the robot maintains a safe and consistent distance from the detected object. A PID (Proportional-Integral-Derivative) controller is used to achieve this, as it effectively minimizes the distance error over time.

#### PID Formula
The velocity is computed using the formula:
\[
v = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{de}{dt}
\]
Where:
- \( K_p \): Proportional gain
- \( K_i \): Integral gain
- \( K_d \): Derivative gain
- \( e \): Distance error

#### PID Implementation in Code
```python
# PID control for velocity
error = e
self.integral += error * self.dt
derivative = (error - self.previous_error) / self.dt
velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
self.previous_error = error

# Limiting velocity
velocity = min(velocity, self.max_velocity)
```

### Lateral Controller: Pure Pursuit

The Pure Pursuit controller is used to adjust the robot's steering by determining a **lookahead point** and calculating the necessary curvature to align the robot's trajectory with the target point.

#### Key Update: Position Calculation
The robot's position is updated in real-time using the relative position data (`data.x` and `data.y`) with respect to the robot's current location and orientation. This ensures the robot's position reflects its instantaneous movement:

```python
x = self.currentx + data.x * math.cos(self.yaw) + data.y * math.sin(self.yaw)
y = self.currenty + data.x * math.sin(self.yaw) - data.y * math.cos(self.yaw)
```

## Pipeline Explanation
### Codes Structure

The pipeline integrates the following functionalities across simulation and real-life environments:

- Frame Publisher: Publishes Kinect or Gazebo camera frames.
- Frame Subscriber & Detection: Subscribes to the frame, applies the SSD detector, and extracts bounding box center points.
- Depth Extraction: Calculates the real-world position (X, Y, Z) of the detected object.
- Error Conversion: Transforms pixel domain errors into real-world distances using camera calibration data.
- Transformation: Converts camera frame coordinates into robot base link coordinates (simulation-specific).
- Controller Feed: Takes the transformed coordinates from the camera and feeds it to the controller to actuate the wheels with specific signals.

## Results
The below gif could simply shows the behavior in simulaton


<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="/Perception/Human_following_from_scratch/Results/clideo_editor_c72baac30f994e27a8708d71057f7e92.gif" alt="First GIF" style="width: 80%;"/>
 
</div>
