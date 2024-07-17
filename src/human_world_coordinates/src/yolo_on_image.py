#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

class ImageProcessor:
    def __init__(self):
        rospy.init_node('yolo_processor', anonymous=True)
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        # self.model.classes = [0]
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.all_objects_pub = rospy.Publisher("/yolo/all_objects_image", Image, queue_size=10)
        self.single_human_pub = rospy.Publisher("/yolo/single_human_image", Image, queue_size=10)
    
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
        
        print("Results:", results[0].boxes)

        # for box in results[0].boxes:
        #     print(box)
        #     # continue
        #     if box.cls == 0 and not human_detected:
        #         x, y, w, h = map(int, box.xywh.cpu())
        #         cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #         human_detected = True
        
        # try:
        #     single_human_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        #     self.single_human_pub.publish(single_human_image)
        # except CvBridgeError as e:
        #     print("Error publishing single human image: ", e)

if __name__ == '__main__':
    ip = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
