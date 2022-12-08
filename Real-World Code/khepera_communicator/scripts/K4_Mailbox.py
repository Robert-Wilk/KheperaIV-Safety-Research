#!/usr/bin/env python3

import rospy
import math
import time
import rosnode
# Import message file
from std_msgs.msg import String
from khepera_communicator.msg import K4_controls, SensorReadings, Opt_Position
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import tf_conversions
from utilities.util import filter_ip

start = time.time()

rospy.init_node('Mailbox', anonymous=True)

# Get all the node names for all the currently running K4_Send_Cmd nodes (all running Kheperas)
# Get the node names of all the current running nodes
node_list = rosnode.get_node_names()

# Find the nodes that contains the "K4_Send_Cmd_" title
khep_node_list = [s for s in node_list if "K4_Send_Cmd_" in s]
ip_num_list = [x[13:16] for x in khep_node_list]
khep_node_cnt = len(khep_node_list)

# there might be leftover underscores due to ips not being 3 digits, so filter
ip_num_list = filter_ip(ip_num_list)

# publish the mailbox once every robot position has been recieved once
pub = rospy.Publisher('K4_Mailbox', Opt_Position, queue_size = 10)
ready = np.zeros(khep_node_cnt, dtype=bool)

# init positions message
positions = Opt_Position()

empty_arr = np.zeros(khep_node_cnt).tolist()

positions.ip = empty_arr.copy()
positions.x = empty_arr.copy()
positions.y = empty_arr.copy()
positions.theta = empty_arr.copy()

def callback(data, args):
    global positions, ready
    # i indicates which subsciber this callback function belongs to,
	# thus it knows which publisher/topic to publish to
    i = args

    theta = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
    positions.ip[i] = int(ip_num_list[i])
    positions.x[i] = data.pose.position.x
    positions.y[i] = data.pose.position.y
    positions.theta[i] = theta
    # print(positions)
    
    ready[i] = True
    # print(i)
    if np.all(ready == True):
        pub.publish(positions)
        ready = np.zeros(khep_node_cnt, dtype=bool)
        # print('sending')

    # rospy.sleep(.25) # This might be the key to getting them to not drive in circles

def central():
    global positions
	# Set up the Subscribers
    print('Set up the Subscribers')
    sub = []
    for i in range(khep_node_cnt):
		# Automatically subscribes to existing vicon topics corresponding to each khepera
        sub.append(rospy.Subscriber('/vrpn_client_node/KhpIV' + ip_num_list[i] + '/pose', PoseStamped, callback, i ))
        positions.ip[i] = ip_num_list[i]
        print(ip_num_list)

    print('Completed Setup')
    # Spin to loop all callback functions
    rospy.spin()


if __name__ == '__main__':
	try:
		central()
	except rospy.ROSInterruptException:
		print(rospy.ROSInterruptException)
