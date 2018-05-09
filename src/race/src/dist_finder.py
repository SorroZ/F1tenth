#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

desired_trajectory = 1
vel = 10
L = 2

pub = rospy.Publisher('error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def callback(data):
	#Variable L used as fix distance to drive with current direction.
	global L
	front_min = 2

	theta = 30
	# Ranges[] contains distances from car to the wall with different angles. Index 180 refers to the distance
	# that always makes an angle of 90 degrees relative the cars direction. The jump between two elements are 0.25
	# degrees. Index 380 = 50 degrees.
	a = data.ranges[300]
	b = data.ranges[180]
	front = data.ranges[720]
	swing = math.radians(theta)

	#Compute angle alpha, distance AB and CD.
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	AB = b*math.cos(alpha)
	CD = AB + L*math.sin(alpha)
	#Converts alpha to degrees, is used to print out on screen.
	alpha_deg = math.degrees(alpha)

	error = CD - desired_trajectory
	print("a: {},   b: {},   alpha: {}, error: {}".format(a,b,alpha_deg,error))

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)


if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
