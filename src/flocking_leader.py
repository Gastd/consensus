#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose as TurtlePose
from tf.transformations import quaternion_from_euler




if __name__ == '__main__':


	# initializes node
	rospy.init_node("flocking_leader", anonymous=True)
	print("leader is on!")
	# starts subscribers and publishers
	# print('subscribers on!')
	pub = rospy.Publisher("/leader/pose", Odometry, queue_size=10)
	pub_yaw = rospy.Publisher("/leader/yaw", Float32, queue_size=10)
	print('publishers on!')



	msg = Odometry()
	yaw = Float32()
	delta_t = 0.01
	rate = rospy.Rate(1/delta_t)
	# state vector = [x, y, theta]
	leader_state = [0,0,0]

	leading_v = 0.5
	leading_theta_speed = 0.25 * delta_t
	# A = 
	leading_theta = np.sin(np.concatenate( (np.arange(0, -np.pi/2, -leading_theta_speed), 
											np.arange(-np.pi/2, np.pi/2, leading_theta_speed), 
											np.arange( np.pi/2, 0, -leading_theta_speed))
											)) * np.pi/4
	# leading_theta = np.concatenate( (leading_theta, -leading_theta) )
	
	i = 0
	cnt = 0
	while not rospy.is_shutdown():

		msg.header.seq = cnt
		msg.header.stamp = rospy.get_rostime()
		msg.header.frame_id = "odom"
		msg.child_frame_id = "/leader/base_link"
		# leader control
		msg.twist.twist.linear.x = leading_v
		odom_quat = quaternion_from_euler(0, 0, leading_theta[i])
		msg.pose.pose.orientation.x = odom_quat[0]
		msg.pose.pose.orientation.y = odom_quat[1]
		msg.pose.pose.orientation.z = odom_quat[2]
		msg.pose.pose.orientation.w = odom_quat[3]

		yaw.data = leading_theta[i]

		# leader state
		# msg.pose.pose.position.x = leading_v * math.cos(leader_state[2]) * delta_t + leader_state[0]
		# msg.pose.pose.position.y = leading_v * math.sin(leader_state[2]) * delta_t + leader_state[1]
		msg.pose.pose.position.z = 0.0

		# update state
		leader_state[0] = msg.pose.pose.position.x
		leader_state[1] = msg.pose.pose.position.y
		leader_state[2] = leading_theta[i]
		i += 1
		cnt += 1

		if i == len(leading_theta):
			i = 0

		pub.publish(msg)
		pub_yaw.publish(yaw)
		rate.sleep()

				
