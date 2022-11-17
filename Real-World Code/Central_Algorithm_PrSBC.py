#!/usr/bin/env python

import rospy
import math
import time
import rosnode
# Import message file
from std_msgs.msg import String
from khepera_communicator.msg import K4_controls, SensorReadings
from geometry_msgs.msg import PoseStamped

import tf_conversions
import tf2_ros
import numpy as np
from qpsolvers import solve_qp

from rps.utilities.controllers import *
from utilities.pr_barrier_certs import *

start = time.time()

rospy.init_node('Central_Algorithm', anonymous=True)

# Get all the node names for all the currently running K4_Send_Cmd nodes (all running Kheperas)
# Get the node names of all the current running nodes
node_list = rosnode.get_node_names()

# Find the nodes that contains the "K4_Send_Cmd_" title
khep_node_list = [s for s in node_list if "K4_Send_Cmd_" in s]
ip_num_list = [x[13:16] for x in khep_node_list]
khep_node_cnt = len(khep_node_list)

# Establish all the publishers to each "K4_controls_" topic, corresponding to each K4_Send_Cmd node, which corresponds to each Khepera robot
pub = []
for i in range(khep_node_cnt):
	pub.append(rospy.Publisher('K4_controls_' + str(ip_num_list[i]), K4_controls, queue_size = 10))

si_goal = np.array([[-1.1, 1.0], [1.0, -1.1]]) # changes based on Optitrack setup
Kp = 10 
Vmax = 200
Wmax = np.pi
flag = 0
V = 0
W = 0

# global subscribers
sub = []

# PrSBC Parameters
safety_radius = 0.20
confidence_level = 0.90

si_barrier_certificate = create_pr_si_barrier_certificate(safety_radius=safety_radius, confidence_level=confidence_level)

# rps functions
si_controller = create_si_position_controller()
uni_controller = create_clf_unicycle_pose_controller()
si_to_uni, uni_to_si = create_si_to_uni_mapping()

# error variables
v_rand_span = 0.005 * np.ones((2, n)) # setting up velocity error range for each robot

x_rand_span_x = 0.02 * np.random.randint(3, 4, (1, n)) # setting up position error range for each robot,
x_rand_span_y = 0.02 * np.random.randint(1, 4, (1, n)) # rand_span serves as the upper bound of uncertainty for each of the robot

x_rand_span_xy = np.concatenate((x_rand_span_x, x_rand_span_y))

def control_for_one_robot(x, y, theta, goal):
	global Kp, flag, Vmax, Wmax, flag, V, W

	print('--------- Control ---------')
	d = math.sqrt(((goal[0] - x) ** 2) + ((goal[1] - y) ** 2))
	print('distance: %s\n' % d)
	
	print('x: %s\ny: %s\ntheta: %s\n' % (x, y, theta))
	pose = np.array([[x], [y], [theta]])
	si_pose = uni_to_si(pose)
	dxi = si_controller(si_pose, goal)
	dxu = si_to_uni(dxi, pose)
	
	# DONT PASS UNCERTAINTY FOR NOW x_rand_span_xy, v_rand_span
	# TODO: figure out how to call this for a decentralized version
	dxu = si_barrier_certificate(dxu, pose)

	if round(d, 3) > 0.05:
		V = dxu[0,0] * 1000
		W = dxu[1,0]
		print('V: %s\nW: %s\n' % (V, W))
		# V = 0
		# W = 0
	else:
		flag = 1
		V = 0
		W = 0

		print('V: %s\nW: %s\n' % (V, W))

# This callback function is where the centralized swarm algorithm, or any algorithm should be
# data is the info subscribed from the vicon node, contains the global position, velocity, etc
# the algorithm placed inside this callback should be published to the K4_controls topics
# which should have the K4_controls message type:
# Angular velocity: ctrl_W
# Linear velocity: ctrl_V
def callback(data, args):
	global V, W
	# i indicates which subsciber this callback function belongs to,
	# thus it knows which publisher/topic to publish to
	i = args


	# The message to be published
	control_msgs = K4_controls()
	
	# convert theta from optitrack quaternion
	theta = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]

	# Publishing
	#rospy.loginfo(control_msgs)
	control_for_one_robot(data.pose.position.x, data.pose.position.y, theta, si_goal)
	control_msgs.ctrl_V = V
	control_msgs.ctrl_W = W

	pub[i].publish(control_msgs)

def central():
	global sub

	# Set up the Subscribers
	sub = []
	for i in range(khep_node_cnt):
		# Automatically subscribes to existing vicon topics corresponding to each khepera
		sub.append(rospy.Subscriber('vrpn_client_ros/KhpIV' + ip_num_list[i] + '/pose', PoseStamped, callback, i ))

	# Spin to loop all callback functions
	rospy.spin()


if __name__ == '__main__':
	try:
		central()
	except rospy.ROSInterruptException:
		pass
