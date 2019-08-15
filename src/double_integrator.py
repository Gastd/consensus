#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose as TurtlePose
from tf.transformations import euler_from_quaternion



global consensus_state
consensus_state = np.zeros([12, 1])
#                    0   1   2   3    4    5   6   7   8   9    10   11
# consensus_state = [x1, x2, x3, x1_, x2_, x3_ y1, y2, y3, y1_, y2_, y3_]

global diff_state
diff_state = np.zeros([9, 1])
#               0   1   2   3       4       5       6   7   8
# diff_state = [v1, v2, v3, theta1, theta2, theta3, w1, w2, w3]

consensus_state_h = consensus_state.copy()

destination = np.array([[3.6],[5.5],[0],[0],[0],[0],[0.0],[0.0],[0],[0],[0],[0]])

print('global variables initialized')


# TO DO
# adicionar offsets de posicao


def update_pose1(msg):
	#print('update_pose1')
	global consensus_state
	global diff_state
	consensus_state[0, 0] = msg.pose.pose.position.x # position in x-axis
	consensus_state[6, 0] = msg.pose.pose.position.y # position in y-axis

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	diff_state[0, 0] = msg.twist.twist.linear.x # linear speed
	diff_state[3, 0] = euler_from_quaternion(quaternion)[2] # orientation n rads
	diff_state[6, 0] = msg.twist.twist.angular.z # angular speed

	consensus_state[3, 0] =  msg.twist.twist.linear.x * math.cos(diff_state[3, 0]) # velocity in x-axis
	consensus_state[9, 0] =  msg.twist.twist.linear.x * math.sin(diff_state[3, 0]) # velocity in x-axis


def update_pose2(msg):
	#print('update_pose2')
	global consensus_state
	global diff_state
	consensus_state[1, 0] = msg.pose.pose.position.x # position in x-axis
	consensus_state[7, 0] = msg.pose.pose.position.y # position in y-axis

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	diff_state[1, 0] = msg.twist.twist.linear.x # linear speed
	diff_state[4, 0] = euler_from_quaternion(quaternion)[2] # orientation n rads
	diff_state[7, 0] = msg.twist.twist.angular.z # angular speed

	consensus_state[4, 0] =  msg.twist.twist.linear.x * math.cos(diff_state[4, 0]) # velocity in x-axis
	consensus_state[10, 0] =  msg.twist.twist.linear.x * math.sin(diff_state[4, 0]) # velocity in x-axis

	# print(consensus_state)

def update_pose3(msg):
	print('update_pose3')
	global consensus_state
	global diff_state
	consensus_state[2, 0] = msg.pose.pose.position.x # position in x-axis
	consensus_state[8, 0] = msg.pose.pose.position.y # position in y-axis

	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)

	diff_state[2, 0] = msg.twist.twist.linear.x # linear speed
	diff_state[5, 0] = euler_from_quaternion(quaternion)[2] # orientation n rads
	diff_state[8, 0] = msg.twist.twist.angular.z # angular speed

	consensus_state[5, 0] =  msg.twist.twist.linear.x * math.cos(diff_state[5, 0]) # velocity in x-axis
	consensus_state[11, 0] =  msg.twist.twist.linear.x * math.sin(diff_state[5, 0]) # velocity in x-axis

	# print(consensus_state)



if __name__ == '__main__':

	# aramis = 0
	# athos = 1
	# porthos = 2

	robot_name = ["aramis", "athos", "porthos"]
	this_robot = robot_name.index(sys.argv[1])

	# initializes node
	rospy.init_node("double_integrator_" + robot_name[this_robot], anonymous=True)
	print(robot_name[this_robot] + " is on!")
	# starts subscribers and publishers
	rospy.Subscriber("/aramis/pose", Odometry, update_pose1)
	rospy.Subscriber("/athos/pose", Odometry, update_pose2)
	rospy.Subscriber("/porthos/pose", Odometry, update_pose3)
	print('subscribers on!')
	pub = rospy.Publisher("/" + robot_name[this_robot] + "/cmd_vel", Twist, queue_size=10)
	print('publishers on!')


	# control variables 
	n = 3
	order = 2

	alpha_x = 10
	alpha_y = 10

	gamma_x = 10
	gamma_y = 10

	k = 1

	#xd = [0, 0, 0]
	#yd = [0, 0, 0]

	d = 0.1
	ms = [20, 20, 20]
	Js = [1, 1, 1]

	m = ms[this_robot]
	J = Js[this_robot]

	# calculates laplacian
	A = np.ones([n, n])
	#A = np.zeros([n, n])
	A[0, 2] = 0
	A[1, 2] = 0
	A[2, 0] = 0
	A[2, 1] = 0

	A = A - np.identity(n)
	A = A * k
	diag = []
	row_sum = np.sum(A, axis=1)
	for i in range(n):
		diag.append(row_sum[i])
	D = np.diag(row_sum)
	L = D - A
	#print(A)
	#print(D)
	#print(L)

	# calculates compact form form Ax
	A_1 = np.concatenate((np.zeros([n, n]), np.identity(3)), axis=1)

	A_2_1 = -(L + alpha_x * np.identity(n))
	A_2_2 = gamma_x * A_2_1
	A_2 = np.concatenate((A_2_1, A_2_2), axis=1)
	Ax = np.concatenate((A_1, A_2), axis=0)

	A_2_1 = -(L + alpha_y * np.identity(n))
	A_2_2 = gamma_y * A_2_1
	A_2 = np.concatenate((A_2_1, A_2_2), axis=1)
	Ay = np.concatenate((A_1, A_2), axis=0)


	Axy1 = np.concatenate((Ax, np.zeros([2*n, 2*n])), axis=1)
	Axy2 = np.concatenate((np.zeros([2*n, 2*n]), Ay), axis=1)
	Axy = np.concatenate((Axy1, Axy2), axis=0)

	#                    0   1   2   3    4    5   6   7   8   9    10   11
	# consensus_state = [x1, x2, x3, x1_, x2_, x3_ y1, y2, y3, y1_, y2_, y3_]

	#                     0    1    2    3     4     5     6    7    8    9     10    11
	# consensus_state_ = [x1_, x2_, x3_, x1__, x2__, x3__, y1_, y2_, y3_, y1__, y2__, y3__]

	#               0   1   2   3       4       5       6   7   8
	# diff_state = [v1, v2, v3, theta1, theta2, theta3, w1, w2, w3]

	cmd_vel = Twist()
	delta_t = 0.1
	rate = rospy.Rate(1/delta_t)

	while not rospy.is_shutdown():

				sin_theta = math.sin(diff_state[n+this_robot, 0])
				cos_theta = math.cos(diff_state[n+this_robot, 0])
				w = diff_state[2*n+this_robot, 0]
				
				# projection for non-holomicity
				for i in range(n):
					consensus_state_h[i, 0] = consensus_state[i, 0] + d*math.cos(diff_state[n+i, 0]) # xh
					consensus_state_h[order*n+i, 0] = consensus_state[order*n+i, 0] + d*math.sin(diff_state[n+i, 0]) # yh
					consensus_state_h[n+i, 0] = consensus_state[n+i, 0] - d*math.sin(diff_state[n+i, 0])*diff_state[2*n+i, 0] # xh_
					consensus_state_h[(order+1)*n+i, 0] = consensus_state[(order+1)*n+i, 0] + d*math.cos(diff_state[n+i, 0]) * diff_state[2*n+i, 0] # yh

				print(consensus_state_h[this_robot, 0])
				print(consensus_state_h[order*n+this_robot, 0])
				print(diff_state[n+this_robot, 0])

				consensus_state_ =  Axy.dot(consensus_state_h - destination)

				ax = consensus_state_[n+this_robot, 0]
				ay = consensus_state_[n*(order+1)+this_robot, 0]
				v = diff_state[this_robot, 0]

				parameters_matrix = np.mat([[m*cos_theta, m*sin_theta] , [-J/d*sin_theta, J/d*cos_theta]])
				input_acc = np.mat([[ax + v*w*sin_theta + d*(w*w)*cos_theta],[ay - v*w*cos_theta + d*(w*w)*sin_theta]])

				FT = parameters_matrix * input_acc

				a = FT[0, 0]/m
				w_ = FT[1, 0]/J

				v = v + a * delta_t
				w = w + w_ * delta_t

				#print(v)
				#print(w)

				if v > 0.1:
					v = 0.1
				elif v < -0.1:
					v = -0.1

				if w > 0.1:
					w = 0.1
				elif w < -0.1:
					w = -0.1

				cmd_vel.linear.x = v
				cmd_vel.angular.z = w

				pub.publish(cmd_vel)
				rate.sleep()

				
