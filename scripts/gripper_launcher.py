#!/usr/bin/env python

import rospy
import rosnode
import rospkg
import subprocess
import time
import os
import argparse
import sys
import yaml

progName = os.path.basename(__file__)

def main(argv):
	
	# Parse the command line arguments 
	parser = argparse.ArgumentParser(
		description='Launch the ROS components of a Gripper Hand')
	
	parser.add_argument('name', nargs='+',
		help="The desired name for the grippers. (Can specify more than one)")
	parser.add_argument('--port', nargs='+', 
		help='The USB ports the hands are attached to. ' +\
		'(defaults to \'/dev/ttyUSB<#>\' starting at 0)')
	parser.add_argument('--nohand', action="store_true", 
		help='Option to set up all the required background nodes but ' +\
		'not the main gripper node (If the user want\'s to launch the ' +\
		'node themselves or is using the RR Bridge)')
	parser.add_argument('--calibrate', action="store_true",
		help='Calibrate the hand as part of the setup script. ' +\
		'If a gripper with the given name has not been started ' +\
		'before the calibration script will be run by default')
	
	args = parser.parse_args(argv)

	calibrate = args.calibrate if args.calibrate else False
	nohand = args.nohand if args.nohand else False

	names = args.name
	ports = args.port
	if ports == None:
		# Default ports to '/dev/ttyUSB<#>''
		ports = ['/dev/ttyUSB%s'%i for i in range(0,len(names))]
	elif len(ports) != len(names):
		print '%s: error: ports incorrectly listed. ' %(progName) +\
		'If --ports is specified a port must be listed for each ' +\
		'gripper specified' 
		return -1

	# Make sure a Master is already running
	print "Checking if Master Node is running..."
	try:
		rospy.has_param("/rosout")
	except Exception, e:
		print e
		print "No Master Found.\nStarting Master..."
		print "(Process will open in a new window)"
		subprocess.call(['gnome-terminal', '-e', "bash -c \"roscore\""])
		time.sleep(2)

	# check if a profile for this name exists yet
	rospack = rospkg.RosPack()
	pkgPath = rospack.get_path('gripper')
	yamlPath = 'yaml'
	for name in names:
		filename = 'gripper_%s_zero_points.yaml' % name
		filePath = os.path.join(pkgPath, yamlPath, filename)	
		# if the name has not been loaded before. 
		# create a default zero points file for it.
		# calibration script will need to be called.
		if not os.path.isfile(filePath):
			calibrate = True
			keys = [('gripper_'+name+'_f1'),
                ('gripper_'+name+'_f2'),
                ('gripper_'+name+'_f3'),
                ('gripper_'+name+'_preshape')]
			zero_points_yaml = {key : dict(zero_point = 0.0) for key in keys}
			with open(filePath, 'w') as outfile:
				outfile.write(yaml.dump(zero_points_yaml))
	
	# Start the nodes for each motor
	print "Starting the services for each motor..."
	print "(Each process will open in a new window)"
	full_command=[]
	for i in range(0, len(names)):
		full_command += ['--tab', '-e', 
			"bash -c \"roslaunch gripper gripper_motors.launch " +\
					   "Name:=%s Port:=%s\"" %(names[i],ports[i])]
	subprocess.call(['gnome-terminal'] + full_command)

	# Make sure the required nodes have successfully launched
	print "Checking for successful launch of all required nodes..."
	time.sleep(3)
	loopCount = 0
	found = False
	for name in names:
		# check that finger was successfully started
		print "\tChecking %s..." % name
		found = "/dynamixel_manager_%s" % name in rosnode.get_node_names() 
		if(not found):
			print "%s: error: It appears that the required " %(progName) +\
			"nodes were not successfully started. Please try again."
			return -1
		loopCount = 0		

	# Calibrate the fingers
	if not calibrate:
		ans = raw_input("Would you like to calibrate the hands? (y/n)\t")
		if ans == 'y':
			calibrate = True
		elif ans == 'n':
			calibrate = False
			print "Skipping calibration..."
		else:
			print "That was not a valid option."
			print "Skipping calibration..." 
	
	if calibrate:
		print "Starting Calibration script..."
		for name in names:
			print "============================================"
			subprocess.call('rosrun gripper calibrate.py ' + name, shell=True)

		print "Calibration Finished"

	if not nohand:
		print "Starting the nodes for each hand..."
		print "(Each process will open a new window)"
		full_command=[]
		for name in names:
			#full_command = "\"rosrun gripper ReflexSFHand.py %s\"" % name
			#subprocess.call(['gnome-terminal', '-e', "bash -c " + full_command])
			full_command += ['--tab', '-e', 'bash -c \"rosrun gripper ReflexSFHand.py %s\"' % name]
		subprocess.call(['gnome-terminal'] + full_command)
			

	raw_input("Setup for all hands is complete.\nPress enter to exit the launch script")
	
if __name__=="__main__":
	main(sys.argv[1:])