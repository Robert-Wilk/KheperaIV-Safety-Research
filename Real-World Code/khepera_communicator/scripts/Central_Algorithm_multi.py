#!/usr/bin/env python3

import rospy
import math
import time
import rosnode
# Import message file
from std_msgs.msg import String
from khepera_communicator.msg import K4_controls, SensorReadings
from geometry_msgs.msg import TransformStamped, PoseStamped

import tf_conversions
import tf2_ros
import numpy as np
from qpsolvers import solve_qp

from rps.utilities.controllers import *
from utilities.util import filter_ip

start = time.time()

rospy.init_node('Central_Algorithm', anonymous=True)

# Get all the node names for all the currently running K4_Send_Cmd nodes (all running Kheperas)
# Get the node names of all the current running nodes
node_list = rosnode.get_node_names()

# Find the nodes that contains the "K4_Send_Cmd_" title
khep_node_list = [s for s in node_list if "K4_Send_Cmd_" in s]
ip_num_list = [x[13:16] for x in khep_node_list]
khep_node_cnt = len(khep_node_list)

# filter ip addresses
ip_num_list = filter_ip(ip_num_list)

# Establish all the publishers to each "K4_controls_" topic, corresponding to each K4_Send_Cmd node, which corresponds to each Khepera robot
pub = []
for i in range(khep_node_cnt):
	pub.append(rospy.Publisher('K4_controls_' + str(ip_num_list[i]), K4_controls, queue_size = 10))

'''
# build goals
x = []
y = []
for i in range(khep_node_cnt):
	if ip_num_list[i] == '104':
		x.append(-1.0)
		y.append(-1.0)
	elif ip_num_list[i] == '177':
		x.append(-1.0)
		y.append(1.0)
	elif ip_num_list[i] == '163':
		x.append(1.0)
		y.append(-1.0)
	elif ip_num_list[i] == '16':
		x.append(1.0)
		y.append(1.0)

# Robot dynamics variables
si_goal = np.array([x, y]) # changes based on Optitrack setup
print(si_goal.shape)
'''
# globals
# si_goal = np.array([[-1.1, -1.0], [1.0, -1.0]])
si_goal = np.array([[1.0, 1.0], [1.0, -1.0]])
# uni_goal = np.array([[-1.1], [1.0], [0]]) # changes based on Optitrack setup

Kp = 10 
Vmax = 200
Wmax = np.pi
flag = np.zeros(khep_node_cnt)
V = 0
W = 0

# global subscribers
sub = []

# rps functions
si_controller = create_si_position_controller()
uni_controller = create_clf_unicycle_pose_controller()
si_to_uni, uni_to_si = create_si_to_uni_mapping()


def control_for_one_robot(x, y, theta, goal, i):
	global Kp, flag, Vmax, Wmax, flag, V, W

	print('--------- Control for %s ---------' % ip_num_list[i])
	goal = np.array([[goal[0]], [goal[1]]])
	d = math.sqrt(((goal[0] - x) ** 2) + ((goal[1] - y) ** 2))
	print('distance: %s\n' % d)
	
	print('x: %s\ny: %s\ntheta: %s\n' % (x, y, theta))
	pose = np.array([[x], [y], [theta]])
	si_pose = uni_to_si(pose)
	dxi = si_controller(si_pose, goal)
	dxu = si_to_uni(dxi, pose)
	
	if round(d, 3) > 0.05:
		V = dxu[0,0] * 1000
		W = dxu[1,0]
		print('V: %s\nW: %s\n' % (V, W))
		# V = 0
		# W = 0
	else:
		flag[i] = 1
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
    global V, W, flag
	# i indicates which subsciber this callback function belongs to,
	# thus it knows which publisher/topic to publish to
    i = args

	# The message to be published
    control_msgs = K4_controls()

	# convert theta from optitrack quaternion
    theta = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
    
	# control robot
    control_for_one_robot(data.pose.position.x, data.pose.position.y, theta, si_goal[:, i], i)
    control_msgs.ctrl_V = V
    control_msgs.ctrl_W = W

    if flag[i] == 1:
        control_msgs.ctrl_V = 0
        control_msgs.ctrl_W = 0
        sub[i].unregister()

	# publish control veloticies
    pub[i].publish(control_msgs)

    # rospy.sleep(.25)


def check_flags():
	global flag

	for i in range(len(flag)):
		if flag[i] == 0:
			return False
			
	return True


def central():
	global sub, flag

	for i in range(khep_node_cnt):
		# Automatically subscribes to existing vicon topics corresponding to each khepera
		print('Created Subscriber: %s\n' % '/vrpn_client_node/KhpIV' + ip_num_list[i] + '/pose')
		sub.append(rospy.Subscriber('/vrpn_client_node/KhpIV' + ip_num_list[i] + '/pose', PoseStamped, callback, i ))

	# rate = rospy.Rate(40)
	# Spin to loop all callback functions
	while not rospy.is_shutdown():
		rospy.sleep(1)
		display = check_flags()
		if display:
			for i in range(khep_node_cnt):
				control_msgs = K4_controls()
				control_msgs.ctrl_V = 0
				control_msgs.ctrl_W = 0
				
				# publish control veloticies
				pub[i].publish(control_msgs)
			print('Program Complete! Please press ^C')

	# print('main finished')

if __name__ == '__main__':
	try:
		central()
	except rospy.ROSInterruptException:
		print(rospy.ROSInterruptException)
