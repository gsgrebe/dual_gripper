#!/usr/bin/env python

import rospy
import subprocess
import time
import os

def main():
	# make sure core is running 
	print "Checking if Master Node is running..."
	try:
		rospy.get_master()
	except Exception, e:
		print "No Master Found.\nStarting Master..."
		print "(Process will open in a new window)"
		subprocess.call(['gnome-terminal', '-e', "bash -c \"roscore\""])

	leftPath = raw_input("Path to LEFT hand (e.g. /dev/ttyUSB0) ==> ")
	rightPath = raw_input("Path to RIGHT hand (e.g. /dev/ttyUSB1) ==> ")
		

	print "Starting the services for each motor..."
	print "(Process will open in a new window)"
	# run the first launch file in a new terminal
	full_command = "\"roslaunch dual_gripper dual_gripper_motors.launch"
	full_command += " leftPort:=" + leftPath
	full_command += " rightPort:=" + rightPath +"\""
	subprocess.call(['gnome-terminal', "-e", "bash -c " + full_command])
	
	# wait a few seconds for this to finish
	time.sleep(5)
	cal = ""
	while True:
		cal = raw_input("Calibrate Hands? [Both='b' LeftOnly='l' RightOnly='r' None='n'] ==> ")
		if cal == 'b':
			#calibrate both
			subprocess.call('rosrun dual_gripper calibrate_dual.py',shell=True)
			break
		
		elif cal == 'l':
			#calibrate left only
			subprocess.call('rosrun dual_gripper calibrate_left.py',shell=True)
			break
		elif cal == 'r':	
			#calibreate right only
			subprocess.call('rosrun dual_gripper calibrate_right.py',shell=True)
			break
		elif cal == 'n':
			break
		else:
			print "That was not a valid response."

	print "Calibration Finished"

	print "Starting services for each hand..."
	subprocess.call(['gnome-terminal', '-e', "bash -c \"rosrun dual_gripper dual_gripper_hands.py\""])

	raw_input("Setup for Both hands is complete.\nPress enter to exit the launch script")

if __name__ == '__main__':
	main()