#!/usr/bin/env python

from gripper import ReflexSFHand
import argparse, sys

if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description="Calibrate a Gripper Hand")
	parser.add_argument('name', help="The name of the gripper hand to calibrate")

	args = parser.parse_args(sys.argv[1:])

	name = args.name
	hand = ReflexSFHand(name)
	hand.calibrate()