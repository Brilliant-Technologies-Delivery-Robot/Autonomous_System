#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32  # Changed from Float32 to Int32
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from tracker import Tracker  # Assuming the Tracker class is defined elsewhere in your code
from sensor_msgs.msg import Image

# Constants
MODEL_PATH = '/home/omar_ben_emad/batekh_ws/src/Autonomous_System/path_planning/kinect_in_reallife/yolov8l-seg.pt'
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
THRESHOLD = 0.65

class YoloSegmentationModel:
    def __init__(self, model_path, threshold, device='cuda'):
        self.model = YOLO(model_path)
        self.threshold = threshold
        self.device = device

    def segment_objects(self, frame):
        return self.model.track(frame, persist=True, conf=self.threshold, device=self.device)

class Visualization:
    def __init__(self, frame_width, frame_height):
        self.frame_width = frame_width
        self.frame_height = frame_height

    def draw_segmentation(self, frame, masks, track_ids, class_ids):
        for mask, track_id, class_id in zip(masks, track_ids, class_ids):
            if class_id == 0:  # Process only 'person'
                color = (0, 0, 255)

                # Calculate bounding box
                x_min, y_min = np.min(mask, axis=0)
                x_max, y_max = np.max(mask, axis=0)

                # Print bounding box coordinates
                print(f"Bounding Box: x1={x_min}, y1={y_min}, x2={x_max}, y2={y_max}")

                # Draw bounding box and label
                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
                label = f"ID:{track_id}"
                cv2.putText(frame, label, (int(x_min), int(y_min) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                # Calculate center
                bbox_center_x = (x_min + x_max) / 2
                bbox_center_y = (y_min + y_max) / 2

                yield bbox_center_x, bbox_center_y

class SegmentationNode:
    def __init__(self, yolo_model, visualizer):
        rospy.init_node('laptop_camera_segmentation', anonymous=True)
        self.bridge = CvBridge()
        self.yolo_model = yolo_model
        self.visualizer = visualizer

        # Initialize the tracker
        self.tracker = Tracker()  # Assuming the Tracker class is defined elsewhere in your code.

        # Publishers
        self.error_x_pub = rospy.Publisher("/error_x", Int32, queue_size=10)  # Changed to Int32
        self.error_y_pub = rospy.Publisher("/error_y", Int32, queue_size=10)  # Changed to Int32
        self.depth_indicator_pub = rospy.Publisher("/depth_indicator", Int32, queue_size=10)  # Changed to Int32
        self.mask_pub = rospy.Publisher("/segmentation/mask", Image, queue_size=10)  # Publisher for mask

        # Initialize OpenCV camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue

            # Perform segmentation
            results = self.yolo_model.segment_objects(frame)
            person_detected = False

            if results and results[0].masks is not None and results[0].boxes is not None:
                masks = results[0].masks.xy
                track_ids = results[0].boxes.id
                class_ids = results[0].boxes.cls

                if track_ids is not None and class_ids is not None:
                    track_ids = track_ids.int().cpu().tolist()
                    class_ids = class_ids.int().cpu().tolist()

                    # Prepare detection for tracking
                    detections = []
                    for mask, track_id, class_id in zip(masks, track_ids, class_ids):
                        if class_id == 0:  # Only track 'person'
                            color = (0, 0, 255)

                            # Get bounding box
                            x_min, y_min = np.min(mask, axis=0)
                            x_max, y_max = np.max(mask, axis=0)

                            # Create detection in (x1, y1, x2, y2, score)
                            detections.append([int(x_min), int(y_min), int(x_max), int(y_max), 1.0])

                            # Create a binary mask image (255 for mask area, 0 for background)
                            mask_image = np.zeros_like(frame[:, :, 0])  # Single channel (grayscale)
                            mask_points = mask.astype(int)
                            cv2.fillPoly(mask_image, [mask_points], 255)  # Fill the mask area with white (255)

                            # Convert the mask to a ROS image message and publish
                            try:
                                mask_msg = self.bridge.cv2_to_imgmsg(mask_image, encoding="mono8")
                                self.mask_pub.publish(mask_msg)  # Publish the mask
                            except Exception as e:
                                rospy.logerr(f"Error converting mask to ROS message: {e}")

                    # Update tracker
                    self.tracker.update(frame, detections)

                    # Draw tracked objects
                    for track in self.tracker.tracks:
                        bbox = track.bbox
                        x1, y1, x2, y2 = bbox
                        track_id = track.track_id

                        color = (0, 0, 255)  # Use a fixed color for now
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)
                        cv2.putText(frame, f'ID {track_id}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                        # Calculate the center for error calculation
                        bbox_center_x = (x1 + x2) / 2
                        bbox_center_y = (y1 + y2) / 2

                        # Calculate errors from frame center
                        error_x = int(bbox_center_x - FRAME_WIDTH / 2)  # Convert to integer
                        error_y = int(bbox_center_y - FRAME_HEIGHT / 2)  # Convert to integer

                        # Publish errors
                        self.error_x_pub.publish(error_x)
                        self.error_y_pub.publish(error_y)

                        # Mark person detection
                        person_detected = True

            # Publish depth indicator (0 for person detected, 1 for no person)
            self.depth_indicator_pub.publish(0 if person_detected else 1)

            # Display the frame with tracking and segmentation
            cv2.imshow('Segmentation with Tracking', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # Initialize YOLO segmentation model and visualization class
        yolo_model = YoloSegmentationModel(MODEL_PATH, THRESHOLD)
        visualizer = Visualization(FRAME_WIDTH, FRAME_HEIGHT)
        
        # Initialize the ROS node
        node = SegmentationNode(yolo_model, visualizer)
        
        # Run the node
        node.run()
        
    except rospy.ROSInterruptException:
        pass
