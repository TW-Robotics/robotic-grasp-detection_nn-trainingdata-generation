#!/usr/bin/env python
import numpy as np
import sys
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d
import random

import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math
from geometry_msgs.msg import Quaternion
from ur5_control import ur5_control

#from capture_pose_calc import capture_pose_calc_structured as poseSampler

debug = False		# Print Debug-Messages

class dataCapture():
	def __init__(self):
		# Init node
		rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

		# Init variables
		self.goals = Pose()			# Store pose fo object

		# Subscriber to Capture-Pose
		rospy.Subscriber("/capturePoses", PoseArray, self.pose_callback, queue_size=1)

		self.ur5 = ur5_control.ur5Controler()

		##################################
		# Give parameters in deg, meters #
		##################################
		# Parameters for randomization
		self.rotateTiltRMin = -10 	# Joint 4: How far to rotate
		self.rotateTiltRMax = 10
		self.rotateUpRMin = -10 	# Joint 5: How far to rotate
		self.rotateUpRMax = 10
		self.rotateRMin = -65		# Joint 6: How far can EEF be rotated
		self.rotateRMax = 65
		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

	# Subscribe to capture-poses
	def pose_callback(self, data):
		self.goals = data

	# Make random moves with last axis
	def move_random(self):
		# Sample random offsets
		rotateUp = random.uniform(0, self.rotateUpRMax)
		rotateDown = random.uniform(self.rotateUpRMin, 0)
		rotateTiltL = random.uniform(0, self.rotateTiltRMax)
		rotateTiltR = random.uniform(self.rotateTiltRMin, 0)

		# Execute offsets
		print_debug("RotUp" + str(rotateUp))
		self.ur5.move_joint(4, rotateUp)
		print_debug("RotD" + str(rotateDown))
		self.ur5.move_joint(4, rotateDown - rotateUp)
		print_debug("TiltL" + str(rotateTiltL))
		self.ur5.move_joint(3, rotateTiltL)
		print_debug("TiltR" + str(rotateTiltR))
		self.ur5.move_joint(3, rotateTiltR - rotateTiltL)

	def capture(self):
		# Drive to the goals and make random moves
		for i in range(len(self.goals.poses)):
			self.ur5.execute_move(self.goals.poses[i])		# Move to base-point
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(0, self.rotateRMax)
			print_debug("Rotating1 " + str(rotateRand))
			self.ur5.move_joint(5, rotateRand)				# Rotate the EEF
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(self.rotateRMin, 0)
			print_debug("Rotating2 " + str(rotateRand))
			self.ur5.move_joint(5, rotateRand)				# Rotate the EEF
			self.move_random()								# Make random moves

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	dc = dataCapture()
	dc.capture()

if __name__ == '__main__':
	main(sys.argv)