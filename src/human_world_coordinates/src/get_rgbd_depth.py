#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from human_world_coordinates.msg import BoundingBox, Distance

from math import *

HEIGHT = 720
WIDTH = 1280

class DepthDistanceNode:
    def __init__(self):
        rospy.init_node('rgbd_depth_distance_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.bounding_box = None
        
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.bbox_sub = rospy.Subscriber('/yolo/bounding_box', BoundingBox, self.bbox_callback)
        self.distance_pub = rospy.Publisher('/rgbd_depth_bb_mid_point', Distance, queue_size=10)
    
    def bbox_callback(self, msg):
        self.bounding_box = msg
    
    def depth_callback(self, msg):
        if self.bounding_box is None:
            return
        
        depth = Distance()

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        x_center = (self.bounding_box.x1 + self.bounding_box.x2) // 2
        y_center = (self.bounding_box.y1 + self.bounding_box.y2) // 2
        
        # Ensure the coordinates are within the image boundaries
        depth_height, depth_width = depth_image.shape

        x_center_depth = x_center * depth_width / WIDTH
        y_center_depth = y_center * depth_height / HEIGHT


        if 0 <= x_center_depth < depth_width and 0 <= y_center_depth < depth_height:
            depth.distance = depth_image[int(floor(y_center_depth)), int(floor(x_center_depth))]
            depth.header.stamp = rospy.Time.now()

            self.distance_pub.publish(depth)
        else:
            rospy.logwarn("Bounding box center is out of depth image bounds.")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DepthDistanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
