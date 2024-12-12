#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from tracker import Tracker

# Constants
MODEL_PATH = '/home/omar_ben_emad/batekh_ws/src/Autonomous_System/path_planning/kinect_in_reallife/yolov8l.pt'
NEEDED_OBJECTS = ["person"]
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
THRESHOLD = 0.65


class YoloModel:
    """Handles YOLOv8 model loading and inference."""
    def __init__(self, model_path, threshold, device='cuda'):
        self.model = YOLO(model_path)
        self.threshold = threshold
        self.device = device

    def detect_objects(self, frame):
        """Performs detection on the provided frame."""
        return self.model(frame, conf=self.threshold, device=self.device)


class Visualization:
    """Handles visualization tasks for detected objects."""
    def __init__(self, frame_width, frame_height):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1
        self.font_thickness = 2
        self.color_static = (0, 255, 0)  # Green
        self.color_dynamic = (0, 0, 255)  # Red
        self.overlay_opacity = 0.5

    def draw_bounding_box(self, img, box, label, color, track_id=None):
        """Draws bounding box and label on the image."""
        x, y, w, h = box
        x, y, w, h = int(x), int(y), int(w), int(h)  # Ensure integer values for bounding box
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        label_text = f'{label}' if track_id is None else f'{label} ID:{track_id}'
        label_size, _ = cv2.getTextSize(label_text, self.font, self.font_scale, self.font_thickness)
        label_w, label_h = label_size
        cv2.rectangle(img, (x, y - label_h - 10), (x + label_w + 10, y), color, -1)
        cv2.putText(img, label_text, (x + 5, y - 5), self.font, self.font_scale, (0, 0, 0), self.font_thickness, cv2.LINE_AA)

    def draw_center_lines(self, frame):
        """Draws semi-transparent center lines on the image."""
        overlay = frame.copy()
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        line_color = (255, 255, 255)  # White

        # Vertical and horizontal center lines
        cv2.line(overlay, (center_x, 0), (center_x, self.frame_height), line_color, 2)
        cv2.line(overlay, (0, center_y), (self.frame_width, center_y), line_color, 2)

        # Center point
        center_point_color = (255, 0, 0)
        cv2.circle(overlay, (center_x, center_y), 10, center_point_color, -1)
        cv2.addWeighted(overlay, self.overlay_opacity, frame, 1 - self.overlay_opacity, 0, frame)

    def display_frame(self, frame):
        """Displays the frame."""
        cv2.imshow('Detected Objects', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


class ObjectDetectionNode:
    """Handles the ROS node, detection, and data publishing."""
    def __init__(self, yolo_model, visualizer):
        rospy.init_node('kinect_object_detection', anonymous=True)
        self.bridge = CvBridge()
        self.yolo_model = yolo_model
        self.visualizer = visualizer
        self.tracker = Tracker()

        self.indication_pub = rospy.Publisher("/depth_indicator", Int16, queue_size=10)
        self.bbox_start_pub = rospy.Publisher('bounding_box_start', Point, queue_size=10)
        self.bbox_end_pub = rospy.Publisher('bounding_box_end', Point, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        results = self.yolo_model.detect_objects(frame)

        detections = []
        person_detected = False

        # Process detection results
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                class_index = box.cls[0].int().item()
                label = self.yolo_model.model.names[class_index]
                confidence = box.conf[0].item()

                if confidence >= THRESHOLD and label in NEEDED_OBJECTS:
                    detections.append([x1, y1, x2, y2, confidence])

        # Update tracker
        self.tracker.update(frame, detections)

        # Visualize tracked objects
        for track in self.tracker.tracks:
            bbox = track.bbox
            x1, y1, x2, y2 = bbox
            track_id = track.track_id

            # Draw bounding box and publish start/end points
            self.visualizer.draw_bounding_box(frame, (x1, y1, x2 - x1, y2 - y1), "person", self.visualizer.color_dynamic, track_id)
            start_point = Point(x=float(x1), y=float(y1), z=0.0)
            end_point = Point(x=float(x2), y=float(y2), z=0.0)
            self.bbox_start_pub.publish(start_point)
            self.bbox_end_pub.publish(end_point)
            person_detected = True

        # Publish indicator message if no person is detected
        if not person_detected:
            self.indication_pub.publish(1)
        else:
            self.indication_pub.publish(0)

        self.visualizer.draw_center_lines(frame)
        self.visualizer.display_frame(frame)


if __name__ == '__main__':
    try:
        yolo_model = YoloModel(MODEL_PATH, THRESHOLD)
        visualizer = Visualization(FRAME_WIDTH, FRAME_HEIGHT)
        ObjectDetectionNode(yolo_model, visualizer)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
