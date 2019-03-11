#!/usr/bin/env python
import numpy as np
import sys
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import random

import rospy
import tf
from geometry_msgs.msg import Pose
import math
from geometry_msgs.msg import Quaternion
from ur5_control import ur5_control

def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    return vec

def calc_polar(vec):
	r = math.sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])
	theta = math.acos(vec[2]/r)
	phi = math.atan2(vec[1], vec[0])
	return r, phi, theta

def calc_cartesian(r, phi, theta):	# phi: horizontal, theta: vertical
	vec = np.array([0., 0., 0.])
	vec[0] = r * math.sin(theta) * math.cos(phi)
	vec[1] = r * math.sin(theta) * math.sin(phi)
	vec[2] = r * math.cos(theta)
	return vec

def main(args):
	rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

	ur5 = ur5_control.ur5Controler()

	# Object frame publisher for testing
	v0 = np.array([0.3, 0.4, -0.15])#np.array([0., 0., -0.2])#
	v1 = np.array([-0.5, 0.2, 0.25])	# point chosen by user
	v01 = np.array([v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]])
	
	# Calc polar coordinates of start point
	r, phi01, theta01 = calc_polar(v01)
	print v01
	print r, phi01*180/math.pi, theta01*180/math.pi

	# Give increments
	phiInc = -10
	thetaInc = 15
	phiInc = phiInc/180.*math.pi
	thetaInc = thetaInc/180.*math.pi

	print phiInc, thetaInc

	v2 = calc_cartesian(r, phi01+phiInc, theta01)
	print v2

	pointsx = []
	pointsy = []
	pointsz = []

	phi = phi01
	theta = theta01

	rRMin = 0#-0.1
	rRMax = 0#0.1
	phiRMin = 0#-5./180.*math.pi
	phiRMax = 0#5./180.*math.pi
	rotateRMin = -65./180.*math.pi
	rotateRMax = 65./180.*math.pi

	for i in range(20):
		if i%15 == 0:
			theta = theta - thetaInc
			phiInc = -phiInc
		else:
			phi = phi + phiInc
		rRand = random.uniform(rRMin, rRMax)
		phiRand = random.uniform(phiRMin, phiRMax)

		vec = calc_cartesian(r+rRand, phi+phiRand, theta)
		pointsx.append(vec[0])
		pointsy.append(vec[1])
		pointsz.append(vec[2])

	goals = []

	#return

	for i in range(len(pointsx)):
		# Negate Coordinates for correct orientation of vector
		vi = [-pointsx[i], -pointsy[i], -pointsz[i]]

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

		# Calculate Quaternion representation of angles and broadcast transforms
		rotateRand = random.uniform(rotateRMin, rotateRMax)
		q = tf.transformations.quaternion_from_euler(xAngle-math.pi/2, zAngle, 0, 'sxzx')
		
		goal = Pose()
		goal.position.x = pointsx[i] + v0[0]
		goal.position.y = pointsy[i] + v0[1]			
		goal.position.z = pointsz[i] + v0[2]
		goal.orientation.x = q[0]
		goal.orientation.y = q[1]
		goal.orientation.z = q[2]
		goal.orientation.w = q[3]

		if ur5.isReachable(goal):
			print "YES " + str(i)

			goals.append(goal)
	print len(goals)
	
	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()
	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	#while not rospy.is_shutdown():
	br.sendTransform((v0[0], v0[1], v0[2]),
					 (0, 0, 0, 1),
					 rospy.Time.now(),
					 "object",
					 "world")
	for i in range(len(goals)):
		br.sendTransform((goals[i].position.x - v0[0], goals[i].position.y - v0[1], goals[i].position.z - v0[2]),
							 (goals[i].orientation.x, goals[i].orientation.y, goals[i].orientation.z, goals[i].orientation.w),
							 rospy.Time.now(),
							 "p" + str(i),
							 "object")
		ur5.execute_move(goals[i])
		#rate.sleep()

if __name__ == '__main__':
	main(sys.argv)