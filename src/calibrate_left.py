#! /usr/bin/env python

from ReflexSFHand import ReflexSFHand

if __name__ == '__main__':
	
	leftHand = ReflexSFHands('left')
	leftHand.calibrate()
