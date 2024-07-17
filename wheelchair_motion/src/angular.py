#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from human_world_coordinates.msg import BoundingBox  # Adjust your_package.msg based on your message definition

# Global variables for image dimensions (example)
image_width = 1280
image_height = 720

def bounding_box_callback(msg):
    global cmd_vel_pub

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
    twist_msg = Twist()
    
    # Adjust linear and angular velocities based on distance
    twist_msg.linear.x = 0.0  # Example linear velocity
    twist_msg.angular.z = -0.001 * (bbox_center_x - image_midpoint_x)  # Example angular velocity
    
    cmd_vel_pub.publish(twist_msg)

def main():
    global cmd_vel_pub
    rospy.init_node('angular_movement')
    
    # Create Twist publisher for cmd_vel topic
    cmd_vel_pub = rospy.Publisher('wheelchair_diff/cmd_vel', Twist, queue_size=10)

    # Subscribe to BoundingBox topic
    rospy.Subscriber('/yolo/bounding_boxes', BoundingBox, bounding_box_callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    main()
