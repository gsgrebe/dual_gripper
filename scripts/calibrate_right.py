#! /usr/bin/env python

from dual_gripper import ReflexSFHand

if __name__ == '__main__':
	rightHand = ReflexSFHand('right')
	rightHand.calibrate()
