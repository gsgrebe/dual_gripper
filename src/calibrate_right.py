#! /usr/bin/env python

from ReflexSFHand import ReflexSFHand

if __name__ == '__main__':
	rightHand = ReflexSFHands('right')
	rightHand.calibrate()