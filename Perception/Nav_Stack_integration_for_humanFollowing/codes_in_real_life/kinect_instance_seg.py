#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from tracker import Tracker
from sensor_msgs.msg import Image

# Constants
MODEL_PATH = '/home/omar_ben_emad/batekh_ws/src/Autonomous_System/path_planning/kinect_in_reallife/yolov8l-seg.pt'
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
THRESHOLD = 0.65
DEVICE = 'cuda'  # Use GPU if available

class YoloSegmentationModel:
    def __init__(self, model_path, threshold, device):
        self.model = YOLO(model_path)
        self.threshold = threshold
        self.device = device

    def segment_objects(self, frame):
        return self.model.track(frame, persist=True, conf=self.threshold, device=self.device)

class SegmentationNode:
    def __init__(self, yolo_model):
        rospy.init_node('camera_segmentation_node', anonymous=True)
        self.bridge = CvBridge()
        self.yolo_model = yolo_model
        self.tracker = Tracker()
        
        # Publishers
        self.error_x_pub = rospy.Publisher("/error_x", Int16, queue_size=10)
        self.error_y_pub = rospy.Publisher("/error_y", Int16, queue_size=10)
        self.depth_indicator_pub = rospy.Publisher("/depth_indicator", Int16, queue_size=10)
        self.mask_pub = rospy.Publisher("/segmentation/mask", Image, queue_size=10)

        # Subscriber
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting ROS image message: {e}")
            return

        overlay = frame.copy()
        results = self.yolo_model.segment_objects(frame)
        person_detected = False

        if results and results[0].masks is not None and results[0].boxes is not None:
            masks = results[0].masks.xy
            track_ids = results[0].boxes.id
            class_ids = results[0].boxes.cls

            if track_ids is not None and class_ids is not None:
                track_ids = track_ids.int().cpu().tolist()
                class_ids = class_ids.int().cpu().tolist()

                detections = []
                for mask, track_id, class_id in zip(masks, track_ids, class_ids):
                    if class_id == 0:  # Only track 'person'
                        x_min, y_min = np.min(mask, axis=0)
                        x_max, y_max = np.max(mask, axis=0)

                        detections.append([int(x_min), int(y_min), int(x_max), int(y_max), 1.0])

                        mask_image = np.zeros_like(frame[:, :, 0])
                        mask_points = mask.astype(int)
                        cv2.fillPoly(mask_image, [mask_points], 255)

                        try:
                            mask_msg = self.bridge.cv2_to_imgmsg(mask_image, encoding="mono8")
                            self.mask_pub.publish(mask_msg)
                        except Exception as e:
                            rospy.logerr(f"Error converting mask to ROS message: {e}")

                self.tracker.update(frame, detections)

                for track in self.tracker.tracks:
                    x1, y1, x2, y2 = track.bbox
                    track_id = track.track_id
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                    cv2.putText(frame, f'ID {track_id}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    bbox_center_x = (x1 + x2) / 2
                    bbox_center_y = (y1 + y2) / 2

                    # Draw center of detected box
                    cv2.circle(frame, (int(bbox_center_x), int(bbox_center_y)), 5, (0, 255, 0), -1)

                    error_x = -(int(bbox_center_x - FRAME_WIDTH / 2))
                    error_y = int(bbox_center_y - FRAME_HEIGHT / 2)

                    self.error_x_pub.publish(error_x)
                    self.error_y_pub.publish(error_y)
                    person_detected = True

        # Draw center of the frame
        frame_center_x = FRAME_WIDTH // 2
        frame_center_y = FRAME_HEIGHT // 2
        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)

        # Draw center lines with 50% opacity
        cv2.line(overlay, (frame_center_x, 0), (frame_center_x, FRAME_HEIGHT), (255, 255, 255), 2)
        cv2.line(overlay, (0, frame_center_y), (FRAME_WIDTH, frame_center_y), (255, 255, 255), 2)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        self.depth_indicator_pub.publish(0 if person_detected else 1)

        cv2.imshow('Segmentation with Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested shutdown")
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        yolo_model = YoloSegmentationModel(MODEL_PATH, THRESHOLD, DEVICE)
        node = SegmentationNode(yolo_model)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
