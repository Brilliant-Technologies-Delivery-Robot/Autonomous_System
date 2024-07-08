import cv2 
import math
import os
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectDetector:
    def __init__(self):
        self.sub_node = 'camera_subscriber'
        self.topic_name = 'obj_detect'
        self.config_file = '/home/omar/delieveryRobotPerception/src/sdd/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        self.frozen_model = '/home/omar/delieveryRobotPerception/src/sdd/frozen_inference_graph.pb'
        self.model = cv2.dnn_DetectionModel(self.frozen_model, self.config_file)
        self.classLabels = []
        self.file_name = '/home/omar/delieveryRobotPerception/src/sdd/labels.txt'
        self.load_labels()

        self.model.setInputSize(320,320)
        self.model.setInputScale(1.0/127.5)
        self.model.setInputMean((127.5,127.5,127.5))
        self.model.setInputSwapRB(True)

        self.font_scale = 2
        self.font = cv2.FONT_HERSHEY_PLAIN

        # Initialize previous bounding box position
        self.prev_bbox = None
        # Set delay interval in seconds
        self.delay_interval = 5  # 5 seconds

    def load_labels(self):
        with open(self.file_name,'rt') as fpt:
            self.classLabels = fpt.read().rstrip('\n').split('\n')

    def callback(self, msg):
        bridgeObj = CvBridge()
        rospy.loginfo("Received Successfuly")
        conv_frame = bridgeObj.imgmsg_to_cv2(msg)
        classIndex, confidence, bbox = self.model.detect(conv_frame, confThreshold=0.65)  # tune the confidence  as required
        conv_frame = cv2.UMat(conv_frame)

        if len(classIndex) != 0:
            for classInd, boxes in zip(classIndex.flatten(), bbox):
                if classInd <= 80:
                    rospy.loginfo(self.classLabels[classInd - 1])

                    cv2.rectangle(conv_frame, boxes, (255, 0, 0), 2)
                    cv2.putText(conv_frame, self.classLabels[classInd - 1], (boxes[0] + 10, boxes[1] + 40), self.font, fontScale=1, color=(0, 255, 0), thickness=2)

                    # Check if previous bounding box exists
                    if self.prev_bbox is not None:
                        # Calculate horizontal distance moved
                        movement = boxes[0] - self.prev_bbox[0]
                        if movement > 0:
                            rospy.loginfo("Bounding box moved to the right")
                        elif movement < 0:
                            rospy.loginfo("Bounding box moved to the left")

                    # Update previous bounding box after delay interval
                    if rospy.get_time() - self.prev_time > self.delay_interval:
                        self.prev_bbox = boxes
                        self.prev_time = rospy.get_time()

        cv2.imshow('frame',conv_frame) 
        cv2.waitKey(1)

    def start_detection(self):
        rospy.init_node(self.sub_node, anonymous=True)
        rospy.Subscriber(self.topic_name, Image, self.callback)
        self.prev_time = rospy.get_time()
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    obj_detector = ObjectDetector()
    obj_detector.start_detection()
