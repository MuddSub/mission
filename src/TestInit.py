#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

rospy.init_node('init')

depthPub = rospy.Publisher("/depth",Float64, queue_size=1, latch=True)
yawPub = rospy.Publisher("/imu/zOrientation", Float64, queue_size=1, latch=True) 

for i in range(20):
	depthPub.publish(0)
	yawPub.publish(0)
