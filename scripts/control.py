#!/usr/bin/python

import rospy
import sys
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time

class TeleRobot:
    def __init__(self):
        # Initialize ROS
        self.node_name = "gesture_control_node"
        rospy.init_node(self.node_name)

	# Parameters for subscirbed and published topic
	gesture_location = rospy.get_param('~/gesture_location_topic')
        gesture_depth = rospy.get_param('~/gesture_depth_topic')
        gesture_detected = rospy.get_param('~/gesture_detected_topic')
	motor_control = rospy.get_param('~/twist_topic')

        # Subscribe and Publish to Topics
        rospy.Subscriber(gesture_location, Float64, self.location_callback)
	rospy.Subscriber(gesture_depth, Int32, self.depth_callback)
	rospy.Subscriber(gesture_detected, String, self.gesture_callback)
	self.pub_twist = rospy.Publisher(motor_control, Twist, queue_size=10, latch=True)

	self.linear = 0
	self.angular = 0
	self.gesture = ""
	
        rospy.loginfo("Publisher Node is loaded...")
	self.r = rospy.Rate(10)
	self.updater()

    def gesture_callback(self, gesture_det):

	self.gesture = gesture_det.data

    def location_callback(self, gesture_loc):

	self.angular = gesture_loc.data

    def depth_callback(self, gesture_dep):
	if gesture_dep.data < 170:
		distance_control = 0
	else:
		distance_control = float(gesture_dep.data-210)/130
	
	self.linear = distance_control

    def updater(self):
	while not rospy.is_shutdown():
		
		if self.gesture == "Rotate":
			self.pub_twist.publish(Twist(Vector3(0,0,0),Vector3(0,0,self.angular))) 
		elif self.gesture == "Follow":
			self.pub_twist.publish(Twist(Vector3(self.linear,0,0),Vector3(0,0,0)))
		else:
			self.pub_twist.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
		
		self.r.sleep()

def main(args):
    	try:
        	TeleRobot()
    	except rospy.ROSInterruptExecption:
        	rospy.loginfo("Shutting down publisher node.")

if __name__ == '__main__':
    main(sys.argv)
