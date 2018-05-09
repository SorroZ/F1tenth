#!/usr/bin/env python

import time
import rospy
import math
from race.msg import drive_param
from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

# Toggle if Uturn should be activated
uturn = False

# Controller parameters
kp = 35.0
kd = 2
servo_offset = 18.5
prev_error = 0.0
vel_input = 25.0
C = 1
I = 0
Ti = 250
Tr = 30

# laser measurements
right_old = 0.0
left_old = 0.0
straight = 0.0

# Controller state and constants
CONTROLLER_STATE = 0
RIGHT = 1
STRAIGHT = 0
LEFT = -1
UTURN1 = 2
UTURN2 = 3
UTURN_WAIT = 4

# angle calculation variables
imu_time_old = 0.0
angle = 0.0
angle_deg = 0.0


iterations = 0
vel_ref = 8

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# This is the main control function. It is triggered when a message is published on
# the topic /error from the node dist_finder. The main feature in this function is the PID controller
# running when the the car is driving straight. If the controller is in a turning mode it will 
# temporary disconnect the PID controller and turn the wheels until the car have turned 
# around the corner. Then the controller goes into PID mode again. Which mode the controller
# should be in is determined with the variable CONTROLLER_STATE.
def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global Ti
	global Tr
	global I
	global C
	global CONTROLLER_STATE
	global STRAIGHT
	global LEFT
	global UTURN1
	global UTURN2
	global UTURN_WAIT
	global RIGHT
	global angle_deg
	global angle
	global vel_ref
	global straight
	
	turn_angle = 55
	spin_angle = 70
	
	# Steer control
	# Decide action depending on CONTROLLER_STATE
	if CONTROLLER_STATE == STRAIGHT:
		steer_signal = C*kp*data.pid_error + I + kd*(prev_error-data.pid_error)
		# steer_signal = C*kp*data.pid_error + kd*(prev_error-data.pid_error)
	elif CONTROLLER_STATE == RIGHT:
		steer_signal = turn_angle
		if angle_deg < -spin_angle:
			CONTROLLER_STATE = STRAIGHT
			I = 0
	elif CONTROLLER_STATE == LEFT:
		steer_signal = -turn_angle
		if angle_deg > spin_angle:
			CONTROLLER_STATE = STRAIGHT
			I = 0
	elif CONTROLLER_STATE == UTURN1:
		steer_signal = -100
		if angle_deg < -70:
			CONTROLLER_STATE = UTURN_WAIT
	elif CONTROLLER_STATE == UTURN_WAIT:
		steer_signal = 0
	elif CONTROLLER_STATE == UTURN2:
		steer_signal = 100
		if angle_deg < -65:
			CONTROLLER_STATE = STRAIGHT
			I = 0

	# velocity control.
	# Decide the speed of the car depending on the state.
	if CONTROLLER_STATE == LEFT or CONTROLLER_STATE == RIGHT:
		vel_ref = 8
	elif CONTROLLER_STATE == UTURN1:
		vel_ref = -18
	elif CONTROLLER_STATE == UTURN_WAIT:
		vel_ref = 0
		# using a sleep is not a good practice but the Uturn is not fully
		# implemented yet.
		time.sleep(5)
		angle = 0
		angle_deg = 0
		CONTROLLER_STATE = UTURN2
	elif CONTROLLER_STATE == UTURN2:
			vel_ref = 8
	elif CONTROLLER_STATE == STRAIGHT:
		# uses the measure of the wall infornt
		# to decide if the car should slow down
		if straight > 6.4:
			vel_ref = 20 
		else:
			vel_ref = 0


	# Saturate the output steering signal if it is to big. 
	if steer_signal > 100:
		steer_signal_out = 100
	elif steer_signal < -100:
		steer_signal_out = -100
	else:
		steer_signal_out = steer_signal
	
	# Output the control signals to the actuators.
	# This is done by publishing a messenge on the topic /drive_parameters
	msg = drive_param();
	vel_input = vel_ref
	msg.velocity = vel_input
	msg.angle = steer_signal_out
	pub.publish(msg)
	print("angle: {} state: {}".format(angle_deg, CONTROLLER_STATE))
	print("steer signal: {},     actual steer signal: {},      I: {}".format(steer_signal, steer_signal_out,I))
	
	# Updates integration part if the PID is active
	# This should always be done after the control signal has been applied.
	if CONTROLLER_STATE == STRAIGHT:
		I = I + (kp/Ti)*data.pid_error + (1/Tr)*(steer_signal_out - steer_signal)
		prev_error = data.pid_error
		

# Helper function to calculate the wall angle.
def calc_wallangle(a,b):
	theta = 30
	swing = math.radians(theta)

	alpha = math.degrees(math.atan((a*math.cos(swing)-b)/(a*math.sin(swing))))
	return alpha

# This function is the callback function when listening to the /scan topic.
# The purpose of this callback function is to set the controller in the 
# right mode depending on the surroundings. It measures distances around
# the car to set the controller in the right mode.
def scan_callback(data):
	global right_old
	global left_old
	global CONTROLLER_STATE
	global LEFT
	global RIGHT
	global STRAIGHT
	global UTURN1
	global UTURN2
	global UTURN_WAIT
	global iterations
	global vel_ref
	global angle_deg
	global angle
	global straight

	a = data.ranges[300]
	b = data.ranges[180]
	wallangle = calc_wallangle(a,b)

	right = data.ranges[360]
	left = data.ranges[720]
	straight = data.ranges[540 - int(round(4*wallangle))]
	
	# detect turns
	if CONTROLLER_STATE == STRAIGHT and iterations > 5:
		if right > 2 * right_old:
			CONTROLLER_STATE = RIGHT
			angle = 0
			angle_deg = 0
		elif left > 2 * left_old:
			CONTROLLER_STATE = LEFT	
			angle = 0
			angle_deg = 0
		elif straight < 0.5 and uturn:
			# using a sleep is not a good practice but the Uturn is not fully
			# implemented yet.
			time.sleep(8)
			CONTROLLER_STATE = UTURN1
			angle = 0
			angle_deg = 0


	
	#print("dist_front: {} state: {} vel_ref: {} angle_deg: {}".format(straight, CONTROLLER_STATE, vel_ref, angle_deg))
	right_old = right
	left_old = left
	iterations += 1

# Callback function from the /imu topic. 
# Continuosly calcluates the angle of the car
def imu_callback(data):
	global imu_time_old
	global angle
	global angle_deg
	#print(data.angular_velocity.z)
	current_time = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
	diff_time = current_time - imu_time_old 
	imu_time_old = current_time	
	angle += data.angular_velocity.z * diff_time
	angle_deg = angle * 180 / math.pi
	

if __name__ == '__main__':
	global kp
	global kd
	global vel_input
	global angle
	global angle_deg
	global vel_ref
	print("Listening to error for PID")
	I = 0
	angle = 0.0
	angle_deg = 0.0
	CONTROLLER_STATE = 0
	iterations = 0
	vel_ref = 0
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.Subscriber("imu", Imu, imu_callback)
	rospy.spin()
