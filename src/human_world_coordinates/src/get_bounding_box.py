#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from human_world_coordinates.msg import BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

from math import *

class ImageProcessor:
    def __init__(self):
        rospy.init_node('yolo_processor', anonymous=True)
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        # self.model.classes = [0]
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.all_objects_pub = rospy.Publisher("/yolo/all_objects_image", Image, queue_size=10)
        self.bounding_boxes_pub = rospy.Publisher("/yolo/bounding_box", BoundingBox, queue_size=10)
    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("Error converting ROS image to OpenCV image: ", e)
            return
        
        results = self.model.track(cv_image, persist=True, classes=[0])
        
        # Publish image with all objects annotated
        try:
            all_objects_image = self.bridge.cv2_to_imgmsg(results[0].plot(), "bgr8")
            self.all_objects_pub.publish(all_objects_image)
        except CvBridgeError as e:
            print("Error publishing all objects image: ", e)
        
        # Publish image with single human annotated
        annotated_frame = cv_image.copy()
        human_detected = False


        # Publish bounding box of human
        for result in results[0].boxes:
            bb_coords_list = [int(floor(i)) for i in (list(result.xyxy)[0]).tolist()]
            human_detected = True
            x1, y1, x2, y2 = bb_coords_list
            # cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            bounding_box = BoundingBox()
            bounding_box.x1 = x1
            bounding_box.y1 = y1
            bounding_box.x2 = x2
            bounding_box.y2 = y2
            bounding_box.header.stamp = rospy.Time.now()
            self.bounding_boxes_pub.publish(bounding_box)
        pass
        

if __name__ == '__main__':
    ip = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
