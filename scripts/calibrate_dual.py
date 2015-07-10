#! /usr/bin/env python

import subprocess

if __name__ == '__main__':
   	subprocess.call('rosrun dual_gripper calibrate_left.py',shell=True)
   	subprocess.call('rosrun dual_gripper calibrate_right.py',shell=True)
