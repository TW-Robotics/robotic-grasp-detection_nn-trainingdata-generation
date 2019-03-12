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

debug = False		# Print Debug-Messages

class capturePoseSampler():
	def __init__(self):
		# Init node
		rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

		# Publisher for Pose-Array
		self.pub = rospy.Publisher('/capturePoses', PoseArray, queue_size=10)
		# Subscriber to Object-Pose
		rospy.Subscriber("/tf_objToBase", Pose, self.objPose_callback, queue_size=1)

		self.ur5 = ur5_control.ur5Controler()

		# Init variables
		self.goals = PoseArray()	# Store all final poses
		objPose = Pose()			# Store pose fo object

		##################################
		# Give parameters in deg, meters #
		##################################
		self.phiInc = -10 		# Horizontal axis angle for base points
		self.thetaInc = 15		# Vertical axis angle for base points
		self.numHorizontal = 15	# After how many phi-increments to change row (increment theta)
		self.thetaMax = 50		# After which thetaAngle to stop generating points

		# Parameters for randomization
		self.rRMin = -0.1 			# How far should poses be nearer/farer from object
		self.rRMax = 0.1
		self.rotateTiltRMin = -10 	# Joint 4: How far to rotate
		self.rotateTiltRMax = 10
		self.rotateUpRMin = -10 	# Joint 5: How far to rotate
		self.rotateUpRMax = 10
		self.rotateRMin = -65		# Joint 6: How far can EEF be rotated
		self.rotateRMax = 65
		self.numRandomGoals = 3		# How many random goals (delta r, phi, theta) should be created
		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

		# Calculate angles in rad
		self.phiInc = self.phiInc/180.*math.pi
		self.thetaInc = self.thetaInc/180.*math.pi
		self.thetaMax = self.thetaMax/180.*math.pi

		# Calbulate necessary parameters for randomization -> can be in range +/- angel/2
		self.phiRMin = -self.phiInc / 2.
		self.phiRMax = self.phiInc / 2.
		self.thetaRMin = -self.thetaInc / 2.
		self.thetaRMax = self.thetaInc / 2.

	# Subscribe to object pose
	def objPose_callback(self, data):
		self.objPose = data

	# Calculate polar coordinates from given cartesian vector	
	def calc_polar(self, vec):
		r = math.sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])
		theta = math.acos(vec[2]/r)
		phi = math.atan2(vec[1], vec[0])
		return r, phi, theta

	# Calculate cartesian coordinates from given polar coordinates
	def calc_cartesian(self, r, phi, theta):	# phi: horizontal, theta: vertical
		vec = np.array([0., 0., 0.])
		vec[0] = r * math.sin(theta) * math.cos(phi)
		vec[1] = r * math.sin(theta) * math.sin(phi)
		vec[2] = r * math.cos(theta)
		return vec

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

	# Calculate full pose to given point w.r.t. object pose
	def get_pose(self, vec):
		# Negate Coordinates for correct orientation of vector
		vi = [-vec[0], -vec[1], -vec[2]]

		# Calculate lengths and angles
		xyLength = math.sqrt(vi[0]*vi[0] + vi[1]*vi[1])
		vecLength = math.sqrt(vi[0]*vi[0] + vi[1]*vi[1] + vi[2]*vi[2])
		zAngle = math.acos(vi[1] / xyLength)
		xAngle = math.acos(xyLength / vecLength)

		# Correct angles in special cases
		if vi[2] < 0:
			#print "xCorr"
			xAngle = -xAngle
		if vi[0] > 0:
			#print "zCorr"
			zAngle = -zAngle
		#print xAngle*180/math.pi, zAngle*180/math.pi

		# Calculate Quaternion representation of angles
		#rotateRand = random.uniform(self.rotateRMin, self.rotateRMax)
		q = tf.transformations.quaternion_from_euler(xAngle-math.pi/2, zAngle, 0, 'sxzx')
		
		goal = Pose()
		goal.position.x = vec[0] + self.objPose.position.x 	# Added objPose to correct pose (Robot calculates with base_link, not with obj_pose)
		goal.position.y = vec[1] + self.objPose.position.y			
		goal.position.z = vec[2] + self.objPose.position.z
		goal.orientation.x = q[0]
		goal.orientation.y = q[1]
		goal.orientation.z = q[2]
		goal.orientation.w = q[3]

		return goal

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	ps = capturePoseSampler()

	# start-point chosen by user TODO -> Read in
	v1 = np.array([0.5, 0.25, 0.45])

	# Calc vector from object to start-point
	v01 = np.array([v1[0]-ps.objPose.position.x, v1[1]-ps.objPose.position.y, v1[2]-ps.objPose.position.z])
	
	# Calc polar coordinates of start point
	r, phi01, theta01 = ps.calc_polar(v01)
	print_debug("Vector given by user: " + str(v01))
	print_debug("Parameters of vector: " + str(r) + str(phi01*180/math.pi) + str(theta01*180/math.pi))

	# Start-Angles for pose-generation
	phi = phi01
	theta = ps.thetaInc

	i = 0
	# Generate Points
	while True:
		# Calculate angles for new point
		if i % ps.numHorizontal == 0 and i != 0:	# Change to next theta (row) after numHorizontal angles
			theta = theta + ps.thetaInc
			ps.phiInc = -ps.phiInc 				# and change orientation of angle-increment to go backwards
		else:
			phi = phi + ps.phiInc 				# Otherwise just go to next angle

		if theta > ps.thetaMax:					# Stop generating points if you reach the final theta angle thetaMax
			break

		# Convert new point into cartesian and calculate the full pose
		vec = ps.calc_cartesian(r, phi, theta)
		goal = ps.get_pose(vec)

		# If the goal is reachable, add it to the goals and make additional random goals
		if ps.ur5.isReachable(goal):
			print_debug("Added point " + str(i))
			ps.goals.poses.append(goal)
			for j in range(ps.numRandomGoals):
				# Get random parameters
				rRand = random.uniform(ps.rRMin, ps.rRMax)
				phiRand = random.uniform(ps.phiRMin, ps.phiRMax)
				thetaRand = random.uniform(ps.thetaRMin, ps.thetaRMax)
				
				# Calculate goal and add it to points if reachable
				vec = ps.calc_cartesian(r+rRand, phi+phiRand, theta+thetaRand)
				goal = ps.get_pose(vec)
				if ps.ur5.isReachable(goal):
					print_debug("Added point " + str(i) + " random " + str(j))
					ps.goals.poses.append(goal)
		i = i + 1
	print "Number of goals generated: " + str(len(ps.goals.poses)) + " = " + str(len(ps.goals.poses) * 4 * 3 + len(ps.goals.poses)) + " poses."	# 4xrandom + 3xrotated + 1 base
	
	rate = rospy.Rate(10)
	for i in range(10):
		# Publish the goals
		ps.pub.publish(ps.goals)
		rate.sleep()

	# Drive to the goals and make random moves
	for i in range(len(ps.goals.poses)):
		ps.ur5.execute_move(ps.goals.poses[i])			# Move to base-point
		ps.move_random()								# Make random moves
		ps.ur5.execute_move(ps.goals.poses[i])			# Move back to base-point

		rotateRand = random.uniform(0, ps.rotateRMax)
		print_debug("Rotating1 " + str(rotateRand))
		ps.ur5.move_joint(5, rotateRand)				# Rotate the EEF
		ps.move_random()								# Make random moves
		ps.ur5.execute_move(ps.goals.poses[i])			# Move back to base-point

		rotateRand = random.uniform(ps.rotateRMin, 0)
		print_debug("Rotating2 " + str(rotateRand))
		ps.ur5.move_joint(5, rotateRand)				# Rotate the EEF
		ps.move_random()								# Make random moves

if __name__ == '__main__':
	main(sys.argv)