#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose as TurtlePose
from tf.transformations import euler_from_quaternion



consensus_state = np.zeros([8, 1])
#                     0   1   2   3       4       5		  6		  7
# consensus_state = [v0, v1, v2, v3, theta0, theta1, theta2, theta3]

ma_size = 1 # moving average size

#edit add offset

print('global variables initialized')


# TO DO
# adicionar offsets de posicao


def update_pose0(msg):
	#print('update_pose1')
	global consensus_state

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	consensus_state[0, 0] = msg.twist.twist.linear.x # linear speed
	consensus_state[4, 0] = euler_from_quaternion(quaternion)[2] # orientation n rads

def update_pose1(msg):
	#print('update_pose1')
	global consensus_state

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	consensus_state[1, 0] = consensus_state[1, 0]*(ma_size-1)/ma_size + msg.twist.twist.linear.x/ma_size # linear speed
	consensus_state[5, 0] = consensus_state[5, 0]*(ma_size-1)/ma_size + euler_from_quaternion(quaternion)[2]/ma_size # orientation n rads



def update_pose2(msg):
	#print('update_pose2')
	global consensus_state

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	consensus_state[2, 0] = consensus_state[2, 0]*(ma_size-1)/ma_size + msg.twist.twist.linear.x/ma_size # linear speed
	consensus_state[6, 0] = consensus_state[6, 0]*(ma_size-1)/ma_size + euler_from_quaternion(quaternion)[2]/ma_size # orientation n rads

	# print(consensus_state)

def update_pose3(msg):
	print('update_pose3')
	global consensus_state

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	consensus_state[3, 0] = consensus_state[3, 0]*(ma_size-1)/ma_size + msg.twist.twist.linear.x/ma_size # linear speed
	consensus_state[7, 0] = consensus_state[7, 0]*(ma_size-1)/ma_size + euler_from_quaternion(quaternion)[2]/ma_size # orientation n rads



if __name__ == '__main__':


	#robots
	robot_name = ["aramis", "athos", "porthos"]
	this_robot = robot_name.index(sys.argv[1])
	n = len(robot_name) + 1


	# initializes node
	rospy.init_node("flocking_" + robot_name[this_robot], anonymous=True)
	print(robot_name[this_robot] + " is on!")
	# starts subscribers and publishers
	rospy.Subscriber("/leader/pose", Odometry, update_pose0)
	rospy.Subscriber("/aramis/pose", Odometry, update_pose1)
	rospy.Subscriber("/athos/pose", Odometry, update_pose2)
	rospy.Subscriber("/porthos/pose", Odometry, update_pose3)
	print('subscribers on!')
	pub = rospy.Publisher("/" + robot_name[this_robot] + "/cmd_vel", Twist, queue_size=10)
	print('publishers on!')


	# control variables 
	k = 0.7
	l = 0.05

	# calculates laplacian
	A = np.array([[0, 0, 0, 0],
				  [1, 0, 1, 1],
				  [1, 1, 0, 1],
				  [1, 1, 1, 0]])

	cmd_vel = Twist()
	delta_t = 0.1
	rate = rospy.Rate(1/delta_t)

	this_robot += 1

	while not rospy.is_shutdown():
				


				u1_sum = 0
				for j in range(n):
					if(j != this_robot):
						v_error = consensus_state[this_robot, 0] - consensus_state[j, 0]
						u1_sum += A[this_robot, j]*v_error

				u1 = -k * u1_sum - l * np.sign(u1_sum)



				u2_sum = 0
				for j in range(n):
					if(j != this_robot):
						theta_error = consensus_state[n + this_robot, 0] - consensus_state[n + j, 0]
						u2_sum += A[this_robot, j]*theta_error

				#u2 = -k * u2_sum - l * np.sign(u2_sum)
				u2 = -k * u2_sum - l * np.arctan(u2_sum)/np.pi*2



				v = consensus_state[this_robot, 0]

				v = v + u1 * delta_t
				w = u2

				cmd_vel.linear.x = v
				cmd_vel.angular.z = w

				pub.publish(cmd_vel)
				rate.sleep()

				
