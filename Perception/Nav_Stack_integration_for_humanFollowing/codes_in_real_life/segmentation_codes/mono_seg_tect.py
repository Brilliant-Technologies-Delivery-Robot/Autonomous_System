#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

# Load YOLOv8 segmentation model
model = YOLO("/home/omar_ben_emad/batekh_ws/src/Autonomous_System/path_planning/kinect_in_reallife/yolov8l-seg.pt")  # segmentation model

# Initialize OpenCV's VideoCapture to use the laptop camera (camera index 0)
cap = cv2.VideoCapture(0)

# Initialize ROS node
rospy.init_node('yolov8_segmentation_node')

# Initialize CvBridge
bridge = CvBridge()

# Publisher for the mask
mask_pub = rospy.Publisher("/segmentation/mask", Image, queue_size=10)

# Check if the camera is opened correctly
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Loop to continuously capture frames from the laptop camera
while True:
    # Read frame from the camera
    ret, im0 = cap.read()

    if not ret:
        print("Error: Failed to capture image from camera.")
        break

    # Initialize Annotator
    annotator = Annotator(im0, line_width=2)

    # Perform object tracking and segmentation
    results = model.track(im0, persist=True)

    # Check if tracking and segmentation are successful
    if results[0].boxes.id is not None and results[0].masks is not None:
        masks = results[0].masks.xy
        track_ids = results[0].boxes.id.int().cpu().tolist()
        class_ids = results[0].boxes.cls.int().cpu().tolist()  # Get class IDs

        # Annotate each person (class_id == 0) with its segmentation mask and tracking ID
        for mask, track_id, class_id in zip(masks, track_ids, class_ids):
            # Only process "person" class (class_id == 0)
            if class_id == 0:  # class ID 0 corresponds to "person"
                color = colors(int(track_id), True)
                txt_color = annotator.get_txt_color(color)
                annotator.seg_bbox(mask=mask, mask_color=color, label=str(track_id), txt_color=txt_color)

                # Draw bounding box (extract the coordinates from the mask)
                x_min = np.min(mask[:, 0])
                y_min = np.min(mask[:, 1])
                x_max = np.max(mask[:, 0])
                y_max = np.max(mask[:, 1])
                cv2.rectangle(im0, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)  # Bounding box

                # Draw the mask (fill the mask area)
                mask_points = mask.astype(int)
                cv2.fillPoly(im0, [mask_points], color)  # Draw the mask over the object

                # Create a binary mask image (255 for mask area, 0 for background)
                mask_image = np.zeros_like(im0[:, :, 0])  # Single channel (grayscale)
                cv2.fillPoly(mask_image, [mask_points], 255)  # Fill the mask area with white (255)

                # Convert the mask to a ROS image message and publish
                try:
                    mask_msg = bridge.cv2_to_imgmsg(mask_image, encoding="mono8")
                    mask_pub.publish(mask_msg)  # Publish the mask
                except Exception as e:
                    rospy.logerr(f"Error converting mask to ROS message: {e}")

    # Display the frame with segmentation
    cv2.imshow("instance-segmentation-person", im0)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
