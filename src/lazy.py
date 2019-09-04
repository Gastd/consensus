#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose as TurtlePose
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64

aramis = [0, 0]
athos = [0, 0]
porthos = [0, 0]

print('global variables initialized')


# TO DO
# adicionar offsets de posicao

def update_pose1(msg):
	#print('update_pose1')
	global aramis

	aramis = [msg.pose.pose.position.x, msg.pose.pose.position.y]


def update_pose2(msg):
	#print('update_pose2')
	global athos

	athos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
	

def update_pose3(msg):
	#print('update_pose3')
	global porthos

	porthos = [msg.pose.pose.position.x, msg.pose.pose.position.y]



if __name__ == '__main__':


	#robots
	robot_name = ["aramis", "athos", "porthos"]
	this_robot = robot_name.index(sys.argv[1])
	n = len(robot_name) + 1


	# initializes node
	rospy.init_node("double_integrator_" + robot_name[this_robot], anonymous=True)
	print(robot_name[this_robot] + " is on!")
	# starts subscribers and publishers
	rospy.Subscriber("/aramis/pose", Odometry, update_pose1)
	rospy.Subscriber("/athos/pose", Odometry, update_pose2)
	rospy.Subscriber("/porthos/pose", Odometry, update_pose3)
	print('subscribers on!')
	aramis_athos = rospy.Publisher("/aramis_athos", Float64, queue_size=10)
	athos_porthos = rospy.Publisher("/athos_porthos", Float64, queue_size=10)
	porthos_aramis = rospy.Publisher("/porthos_aramis", Float64, queue_size=10)
	print('publishers on!')


	while not rospy.is_shutdown():

				msg_aramis_athos = Float64()
				msg_athos_porthos = Float64()
				msg_porthos_aramis = Float64()


				msg_aramis_athos = math.sqrt((aramis[0]-athos[0])**2 + (aramis[1]-athos[1])**2)
				msg_athos_porthos = math.sqrt((athos[0]-porthos[0])**2 + (athos[1]-porthos[1])**2)
				msg_porthos_aramis = math.sqrt((porthos[0]-aramis[0])**2 + (porthos[1]-aramis[1])**2)


				aramis_athos.pub(msg_aramis_athos)
				athos_porthos.pub(msg_athos_porthos)
				porthos_aramis.pub(msg_porthos_aramis)	

				rate.sleep()

				