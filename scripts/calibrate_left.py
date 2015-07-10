#! /usr/bin/env python

from dual_gripper import ReflexSFHand

if __name__ == '__main__':
	
	leftHand = ReflexSFHand('left')
	leftHand.calibrate()
