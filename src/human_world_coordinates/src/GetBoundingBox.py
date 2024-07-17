#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from human_world_coordinates.msg import BoundingBox  # Replace 'custom_msgs' with your package name
from cv_bridge import CvBridge, CvBridgeError

class YoloHumanDetector:
    def __init__(self):
        rospy.init_node('yolo_human_detector', anonymous=True)

        # Define the publishers
        self.bbox_pub = rospy.Publisher('/detected_human/bounding_box', BoundingBox, queue_size=10)
        self.image_pub = rospy.Publisher('/detected_human/image', Image, queue_size=10)
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Load YOLO
        self.net = cv2.dnn.readNet("weights/yolov3.weights", "weights/yolov3.cfg")
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # Load the COCO class labels
        with open("coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
            return

        # Process the image with YOLO
        height, width, channels = cv_image.shape

        # Prepare the frame for YOLO
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Initialize lists for detected bounding boxes, confidences, and class IDs
        boxes = []
        confidences = []
        class_ids = []

        # Loop over each detection
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if self.classes[class_id] == "person" and confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Maximum Suppression (NMS) to filter out weak detections
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        if len(indexes) > 0:
            # Pick the first detected human
            i = indexes[0]
            x, y, w, h = boxes[i]
            label = str(self.classes[class_ids[i]])
            confidence = confidences[i]

            # Draw bounding box
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish bounding box
            bbox_msg = BoundingBox(x=x, y=y, width=w, height=h)
            self.bbox_pub.publish(bbox_msg)

            # Publish image with bounding box
            try:
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Error converting image: {e}")

        # Display the resulting frame (optional)
        cv2.imshow("YOLO Human Detector", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = YoloHumanDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
