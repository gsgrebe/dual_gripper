#!/usr/bin/env python

import rospy
import roslaunch

def main():
	launcher = roslaunch.ROSLaunch()
	launcher.start()
	
	leftHand = roslaunch.Node(package="dual_gripper", node_type="ReflexSFHand.py", name="dual_gripper_left", args="left", output="screen")
	rightHand = roslaunch.Node(package="dual_gripper", node_type="ReflexSFHand.py", name="dual_gripper_right", args="right", output="screen")
	
	launcher.launch(leftHand)
	launcher.launch(rightHand)
	
	launcher.spin()

if __name__ == '__main__':
	main()
