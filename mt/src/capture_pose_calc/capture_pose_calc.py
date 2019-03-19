#!/usr/bin/env python
import numpy as np
import sys
import random
import rospy
import tf
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from ur5_control import ur5_control

debug = True		# Print Debug-Messages

class capturePoseSampler():
	def __init__(self):
		# Init node
		rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

		# Init variables
		self.goals = PoseArray()	# Store all final poses
		self.objCamPose = Pose()

		# Publisher for Pose-Array
		self.pub = rospy.Publisher('/capturePoses', PoseArray, queue_size=10)
		
		# Subscriber to Object-Pose
		rospy.Subscriber("/tf_objImgCenterToCam", Pose, self.objCamPose_callback, queue_size=1)

		self.ur5 = ur5_control.ur5Controler()

		##################################
		# Give parameters in deg, meters #
		##################################
		self.phiInc = -10 		# Horizontal axis angle for base points
		self.thetaInc = 15		# Vertical axis angle for base points
		self.numHorizontal = 15	# After how many phi-increments to change row (increment theta)
		self.thetaMax = 50		# After which thetaAngle to stop generating points

		# Parameters for randomization
		self.rRMin = 0#-0.1 			# How far should poses be nearer/farer from object
		self.rRMax = 0#0.1
		self.numRandomGoals = 0#3		# How many random goals (delta r, phi, theta) should be created
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
	def objCamPose_callback(self, data):
		self.objCamPose = data

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
		q = tf.transformations.quaternion_from_euler(xAngle-math.pi/2, zAngle, 0, 'sxzx')
		
		goal = Pose()
		goal.position.x = vec[0]
		goal.position.y = vec[1]			
		goal.position.z = vec[2]
		goal.orientation.x = q[0]
		goal.orientation.y = q[1]
		goal.orientation.z = q[2]
		goal.orientation.w = q[3]

		return goal

	def calc_poses(self):
		# Wait for transformations to arrive
		while True:
			if self.objCamPose.position.x == 0 and self.objCamPose.position.y == 0 and self.objCamPose.position.z == 0:
				print "Waiting for object-to-cam-transform to arrive..."
				rospy.sleep(1)
			else:
				break

		# Start-point chosen by user = actual pose
		v01 = np.array([self.objCamPose.position.x, self.objCamPose.position.y, self.objCamPose.position.z])
		print_debug("Vector read in from actual pose: " + str(v1))

		# Calc polar coordinates of start point
		r, phi01, theta01 = self.calc_polar(v01)
		print_debug("Parameters of vector: r=" + str(r) + " phi=" + str(phi01*180/math.pi) + " theta=" + str(theta01*180/math.pi))

		# Start-Angles for pose-generation
		phi = phi01
		theta = self.thetaInc

		''' # DEBUG - Append actual point
		vec = self.calc_cartesian(r, phi01, theta01)
		print_debug("Vector calculated: " + str(vec))
		goal = self.get_pose(vec)
		print_debug("Pose calculated: " + str(goal))
		self.goals.poses.append(goal)'''

		i = 0
		# Generate Points
		while True:
			# Calculate angles for new point
			if i % self.numHorizontal == 0 and i != 0:	# Change to next theta (row) after numHorizontal angles
				theta = theta + self.thetaInc
				self.phiInc = -self.phiInc 				# and change orientation of angle-increment to go backwards
			else:
				phi = phi + self.phiInc 				# Otherwise just go to next angle

			if theta > self.thetaMax:					# Stop generating points if you reach the final theta angle thetaMax
				break

			# Convert new point into cartesian and calculate the full pose
			vec = self.calc_cartesian(r, phi, theta)
			goal = self.get_pose(vec)
			print_debug("Vector calculated for position " + str(i) + ": " + str(vec))

			# If the goal is reachable, add it to the goals and make additional random goals
			if self.ur5.isReachable(goal):
				print_debug("Added point " + str(i))
				self.goals.poses.append(goal)
				# Make additional random goals
				for j in range(self.numRandomGoals):
					# Get random parameters
					rRand = random.uniform(self.rRMin, self.rRMax)
					phiRand = random.uniform(self.phiRMin, self.phiRMax)
					thetaRand = random.uniform(self.thetaRMin, self.thetaRMax)
					
					# Calculate goal and add it to points if reachable
					vec = self.calc_cartesian(r+rRand, phi+phiRand, theta+thetaRand)
					goal = self.get_pose(vec)
					if self.ur5.isReachable(goal):
						print_debug("Added point " + str(i) + " random " + str(j))
						self.goals.poses.append(goal)
			i = i + 1
		print "Number of goals generated: " + str(len(self.goals.poses)) + " = " + str(len(self.goals.poses) * 5 * 3) + " poses."	# 5x = 1xBase + 4xMoveRandom in data_capture; 3x = Rotation around last axis in data_capture
		print "Store poses by typing 'rosbag record /capturePoses' and replay them by typing 'rosbag play -l'."
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Publish the goals
			self.pub.publish(self.goals)
			rate.sleep()

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	ps = capturePoseSampler()
	ps.calc_poses()

if __name__ == '__main__':
	main(sys.argv)