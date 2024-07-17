#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DistanceToTwistNode:
    def __init__(self):
        rospy.init_node('distance_to_twist_node', anonymous=True)
        
        self.linear_speed = 0.5  # constant linear speed in m/s if distance >= 3m
        self.distance_sub = rospy.Subscriber('/rgbd_depth_bb_mid_point', Float32, self.distance_callback)
        self.cmd_vel_pub = rospy.Publisher('wheelchair_diff/cmd_vel', Twist, queue_size=10)
	
    def distance_callback(self, msg):
        distance = msg.data
        twist_msg = Twist()
        
        if distance >= 3000:
            twist_msg.linear.x = self.linear_speed
        else:
            twist_msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DistanceToTwistNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
