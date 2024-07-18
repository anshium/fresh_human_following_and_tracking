#!/usr/bin/env python3

# Shoutout to ChatGPT for the sync code

import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from human_world_coordinates.msg import BoundingBox, Distance
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

image_width = 1280
image_height = 720

def callback(bbox_msg, depth_msg):
    x1 = bbox_msg.x1
    y1 = bbox_msg.y1
    x2 = bbox_msg.x2
    y2 = bbox_msg.y2
    depth_value = depth_msg.distance

    twist_msg = Twist()

    bbox_center_x = (x1 + x2) / 2
    bbox_center_y = (y1 + y2) / 2
    
    image_midpoint_x = image_width / 2
    image_midpoint_y = image_height / 2


    max_linear_speed = 0.20
    
    if depth_value >= 3000:
        twist_msg.linear.x = max_linear_speed
    elif depth_value >= 2400:
        twist_msg.linear.x = max_linear_speed - 0.05
    elif depth_value >= 1800:
        twist_msg.linear.x = max_linear_speed - 0.1
    elif depth_value >= 1000:
        twist_msg.linear.x = max_linear_speed - 0.15
    else:
        twist_msg.linear.x = 0
    
    twist_msg.angular.z = -0.001 * (bbox_center_x - image_midpoint_x)

    cmd_vel_pub.publish(twist_msg)

rospy.init_node('sync_publisher')

bbox_sub = Subscriber('/yolo/bounding_box', BoundingBox)
depth_sub = Subscriber('/rgbd_depth_bb_mid_point', Distance)

# Synchronizer
ats = ApproximateTimeSynchronizer([bbox_sub, depth_sub], queue_size=10, slop=0.1)
ats.registerCallback(callback)

cmd_vel_pub = rospy.Publisher('wheelchair_diff/cmd_vel', Twist, queue_size=10)

rospy.spin()