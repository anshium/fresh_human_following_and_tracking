#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from human_world_coordinates.msg import BoundingBox 
from std_msgs.msg import Float32

# Global variables for image dimensions
image_width = 1280
image_height = 720

def bounding_box_callback(msg):
    global cmd_vel_pub, twist_msg

    x1 = msg.x1
    y1 = msg.y1
    x2 = msg.x2
    y2 = msg.y2
    
    # Calculate bounding box center
    bbox_center_x = (x1 + x2) / 2
    bbox_center_y = (y1 + y2) / 2
    
    # Calculate image midpoint
    image_midpoint_x = image_width / 2
    image_midpoint_y = image_height / 2
    
    # Calculate distance
    distance = ((bbox_center_x - image_midpoint_x)**2 + (bbox_center_y - image_midpoint_y)**2)**0.5
    
    # Publish Twist message
    
    
    twist_msg.linear.x = 0.0 
    twist_msg.angular.z = round(-0.001 * (bbox_center_x - image_midpoint_x), 4)
    
    cmd_vel_pub.publish(twist_msg)

def distance_callback(msg):
    global cmd_vel_pub, linear_speed, distance_sub, cmd_vel_pub, twist_msg

    distance = msg.data
    
    twist_msg.linear.x = round(0.005 * (distance / 100), 4)
    
    cmd_vel_pub.publish(twist_msg)

def main():
    global cmd_vel_pub, linear_speed, distance_sub, cmd_vel_pub, twist_msg
    rospy.init_node('angular_movement')
    
    # Create Twist publisher for cmd_vel topic
    cmd_vel_pub = rospy.Publisher('wheelchair_diff/cmd_vel', Twist, queue_size=10)

    twist_msg = Twist()

    linear_speed = 0.15  # constant linear speed in m/s if distance >= 3m
    distance_sub = rospy.Subscriber('/rgbd_depth_bb_mid_point', Float32, distance_callback)


    # Subscribe to BoundingBox topic
    rospy.Subscriber('/yolo/bounding_box', BoundingBox, bounding_box_callback)
    
    cmd_vel_pub.publish(twist_msg)
    
    rospy.spin()

if __name__ == '__main__':
    main()
