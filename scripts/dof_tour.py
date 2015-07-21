#!/usr/bin/env python

import rospy
 
from gripper.msg import Pose
 
def dofTour():

	leftPub = rospy.Publisher('/gripper_left/command', Pose, queue_size=10)
	rightPub = rospy.Publisher('/gripper_right/command', Pose, queue_size=10)
	rospy.init_node('gripper_dof_tour')

	FINGER_CLOSED = 4.6
	FINGER_PINCH = 3.5 

	PRESHAPE_CYLINDER = 0
	PRESHAPE_SPHERICAL = 1.5
	PRESHAPE_PINCH = 2.5

	# finger 1 (right-hand index) close
	rospy.sleep(5)
	leftPub.publish(0,0,0,0)
	rightPub.publish(0,0,0,0)
	rospy.sleep(1)
	leftPub.publish(FINGER_CLOSED, 0, 0, 0)
	rightPub.publish(FINGER_CLOSED,0, 0, 0)
	rospy.sleep(1)
	leftPub.publish(0, 0, 0, 0)
	rightPub.publish(0, 0, 0, 0)
	rospy.sleep(1)
	
	# finger 2 (right-hand middle) close
	leftPub.publish(0, FINGER_CLOSED, 0, 0)
	rightPub.publish(0, FINGER_CLOSED, 0, 0)
	rospy.sleep(1)
	leftPub.publish(0, 0, 0, 0)
	rightPub.publish(0, 0, 0, 0)
	rospy.sleep(1)
	# finger 3 (thumb) close
	leftPub.publish(0, 0, FINGER_CLOSED, 0)
	rightPub.publish(0, 0, FINGER_CLOSED, 0)
	rospy.sleep(1)
	leftPub.publish(0, 0, 0, 0)
	rightPub.publish(0, 0, 0, 0)
	rospy.sleep(1)
	# preshape
	leftPub.publish(0, 0, 0, PRESHAPE_SPHERICAL)
	rightPub.publish(0, 0, 0, PRESHAPE_SPHERICAL)
	rospy.sleep(1)
	leftPub.publish(0, 0, 0, PRESHAPE_PINCH)
	rightPub.publish(0, 0, 0, PRESHAPE_PINCH)
	rospy.sleep(1)
	leftPub.publish(0, 0, 0, 0)
	rightPub.publish(0, 0, 0, 0)
	rospy.sleep(1)
	# hand closed in cylindrical power grasp
	leftPub.publish(FINGER_CLOSED, FINGER_CLOSED, FINGER_CLOSED, 0)
	rightPub.publish(FINGER_CLOSED, FINGER_CLOSED, FINGER_CLOSED, 0)
	rospy.sleep(1)
	# hand open
	leftPub.publish(0, 0, 0, 0)
	rightPub.publish(0, 0, 0, 0)
	rospy.sleep(1)
	# preshape hand for pinch
	leftPub.publish(0, 0, 0, PRESHAPE_PINCH)
	rightPub.publish(0, 0, 0, PRESHAPE_PINCH)
	rospy.sleep(1)
	# pinch grasp
	leftPub.publish(FINGER_PINCH, FINGER_PINCH, 0, PRESHAPE_PINCH)
	rightPub.publish(FINGER_PINCH, FINGER_PINCH, 0, PRESHAPE_PINCH)
	rospy.sleep(1)
	# hand open (pinch grasp)
	leftPub.publish(0, 0, 0, PRESHAPE_PINCH)
	rightPub.publish(0, 0, 0, PRESHAPE_PINCH)
	rospy.sleep(1)
	# hand open (cylindrical grasp)
	leftPub.publish(0, 0, 0, 0)
	rightPub.publish(0, 0, 0, 0)
	rospy.sleep(1)

if __name__ == '__main__':
	try:
		dofTour()
	except rospy.ROSInterruptException:
		pass
