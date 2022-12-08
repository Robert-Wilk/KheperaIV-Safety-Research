#!/usr/bin/env python

import rospy
import math
import time
import rosnode
# Import message file
from std_msgs.msg import String
from khepera_communicator.msg import K4_controls, SensorReadings, Opt_Position
from geometry_msgs.msg import PoseStamped

import tf_conversions
import tf2_ros
import numpy as np
from qpsolvers import solve_qp

# from rps.utilities.controllers import *
from utilities.pr_barrier_certs import *
from utilities.controllers import *
from utilities.util import filter_ip

start = time.time()

rospy.init_node('Algorithm_PrSBC_Central', anonymous=True)

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
pub = {}
for i in range(khep_node_cnt):
	pub.update({str(ip_num_list[i]) : rospy.Publisher('K4_controls_' + str(ip_num_list[i]), K4_controls, queue_size = 10)})

# build goals
x = []
y = []
theta = []
for i in range(khep_node_cnt):
	# 16 and 104 are flipped
	# 163 and 177 are flipped
	if ip_num_list[i] == '16':
		x.append(-1.0)
		y.append(-1.0)
	elif ip_num_list[i] == '163':
		x.append(-1.0)
		y.append(1.0)
	elif ip_num_list[i] == '177':
		x.append(1.0)
		y.append(-1.0)
	elif ip_num_list[i] == '104':
		x.append(1.0)
		y.append(1.0)
	theta.append(0)

# Robot dynamics variables
uni_goals = np.array([x, y, theta]) # changes based on Optitrack setup
Kp = 10 # TODO: might need to scale output depending on how large output of QP is 
Vmax = 200
Wmax = np.pi
flag = np.zeros(khep_node_cnt) # stop flag

# PrSBC Parameters
safety_radius = 0.20
confidence_level = 0.90

# base controller and PrSBC creator
uni_barrier_cert = create_pr_unicycle_barrier_certificate_cent(safety_radius=safety_radius, confidence_level=confidence_level)
position_uni_clf_controller = create_clf_unicycle_pose_controller()

# error/uncertainty variables
'''
v_rand_span = 0.005 * np.ones((2, n)) # setting up velocity error range for each robot

x_rand_span_x = 0.02 * np.random.randint(3, 4, (1, n)) # setting up position error range for each robot,
x_rand_span_y = 0.02 * np.random.randint(1, 4, (1, n)) # rand_span serves as the upper bound of uncertainty for each of the robot

x_rand_span_xy = np.concatenate((x_rand_span_x, x_rand_span_y))
'''
def control_robots(poses, goals):
	
    x = poses

	# TODO: Add noise

    # get distance to gaol for all robots
    d = np.sqrt((goals[0] - x[0]) ** 2 + (goals[1] - x[1]) ** 2)
    print(d)

    # stop if distance threshold is met
    if (d < .05).all() or (flag == 1).all():
        V = np.zeros(khep_node_cnt)
        W = np.zeros(khep_node_cnt)
        sub.unregister()
        return V, W

    # Use a position controller to drive to the goal position
    dxu = position_uni_clf_controller(x, goals)

    dxu = uni_barrier_cert(dxu, x) # DONT PASS UNCERTAINTY FOR NOW x_rand_span_xy, v_rand_span

    for i in range(khep_node_cnt):
        if round(d[i], 2) < .05 or flag[i] == 1:
            dxu[0, i] = 0
            dxu[1, i] = 0
            flag[i] = 1
        else:
            dxu[0, i] *= 1000
            if dxu[0, i] > 200:
                dxu[0, i] = 200

    return dxu[0], dxu[1]

# This callback function is where the centralized swarm algorithm, or any algorithm should be
# data is the info subscribed from the vicon node, contains the global position, velocity, etc
# the algorithm placed inside this callback should be published to the K4_controls topics
# which should have the K4_controls message type:
# Angular velocity: ctrl_W
# Linear velocity: ctrl_V
def callback(data):
	# arrange positions and goals into PrSBC form
	
	# call control_robots with arranged positions/goals and return V and W list
	positions = np.array([data.x, data.y, data.theta])
	V, W = control_robots(positions, uni_goals)

	# send V and W list
	for i in range(khep_node_cnt):
		# create control message
		control_msgs = K4_controls()
		
		# set appropriate velocities
		control_msgs.ctrl_V = V[i]
		control_msgs.ctrl_W = W[i]

		# send data
		print('-------------------------')
		print('Control for robot %s' % data.ip[i])
		print(control_msgs)
		pub.get(str(data.ip[i])).publish(control_msgs)
		#if flag[i] == 1:
		#	print('robot %s has been stopped' % ip_num_list[i])
	rospy.sleep(.25)

def central():
	global sub

	# Set up the Subscribers
	print('Set up the Subscribers')
	sub = rospy.Subscriber('/K4_Mailbox', Opt_Position, callback)
	
	print('Entering Waiting loop')
	while not rospy.is_shutdown():
		rospy.sleep(1)
		if ((flag == 1).all()):
			print('Program Complete! Please press ^C')


if __name__ == '__main__':
	try:
		central()
	except rospy.ROSInterruptException:
		print(rospy.ROSInterruptException)
