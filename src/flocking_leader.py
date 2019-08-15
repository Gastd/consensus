#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose as TurtlePose
from tf.transformations import quaternion_from_euler




if __name__ == '__main__':


	# initializes node
	rospy.init_node("flocking_leader", anonymous=True)
	print(robot_name[this_robot] + " is on!")
	# starts subscribers and publishers
	print('subscribers on!')
	pub = rospy.Publisher("/leader/cmd_vel", Twist, queue_size=10)
	print('publishers on!')


	leading_v = 1
	leading_theta = np.sin(np.arange(0, np.pi, 0.01))


	msg = Odometry()
	delta_t = 0.1
	rate = rospy.Rate(1/delta_t)
	
	i = 0
	while not rospy.is_shutdown():

		msg.twist.twist.linear.x = v
		msg.pose.pose.orientation = quaternion_from_euler(0, 0, leading_theta[i])
		i +=1

		if i == len(leading_theta):
			i = 0

		pub.publish(cmd_vel)
		rate.sleep()

				