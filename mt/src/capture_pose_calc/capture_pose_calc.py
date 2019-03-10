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
import ur5_control

def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    return vec

def main(args):
	rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

	ur5 = ur5_control.ur5Controler()

	# Parameters for random points
	phi = np.linspace(0, np.pi, 20)
	theta = np.linspace(0, 2 * np.pi, 40)
	ri = 0.3
	ro = 0.85
	numPoints = 300
	x = np.outer(np.sin(theta), np.cos(phi)) * ro
	y = np.outer(np.sin(theta), np.sin(phi)) * ro
	z = np.outer(np.cos(theta), np.ones_like(phi)) * ro

	# Object frame publisher for testing
	v0 = np.array([0.3, 0.4, -0.15])

	pointsx = []
	pointsy = []
	pointsz = []

	# Calculate random points
	xi, yi, zi = sample_spherical(numPoints)
	for i in range(numPoints):
		r = random.uniform(ri, ro)
		if i%7 == 0:
			r = ro
		xi[i] = xi[i]*r
		yi[i] = yi[i]*r
		zi[i] = zi[i]*r

	# Cut off points 
	vi0 = []
	for i in range(len(xi)):
		vi0 = np.asarray([xi[i]+v0[0], yi[i]+v0[1], zi[i]+v0[2]])
		vi0N = math.sqrt(vi0[0]*vi0[0] + vi0[1]*vi0[1] + vi0[2]*vi0[2])
		#print vi0N
		if zi[i] > (-r/4) and vi0N < 0.9:
			pointsx.append(xi[i])
			pointsy.append(yi[i])
			pointsz.append(zi[i])

	# Plot points
	fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d', 'aspect':'equal'})
	ax.plot_wireframe(x, y, z, color='k', rstride=1, cstride=1)
	ax.scatter(pointsx, pointsy, pointsz, s=100, c='r', zorder=10)
	plt.show()

	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()
	
	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
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
			q = tf.transformations.quaternion_from_euler(xAngle - math.pi/2, 0, zAngle)
			
			goal = Pose()
			goal.position.x = pointsx[i]
			goal.position.y = pointsy[i]			
			goal.position.z = pointsz[i]
			goal.orientation.x = q[0]
			goal.orientation.y = q[1]
			goal.orientation.z = q[2]
			goal.orientation.w = q[3]

			if ur5.isReachable(goal):
				br.sendTransform((pointsx[i], pointsy[i], pointsz[i]),
								 (q[0], q[1], q[2], q[3]),
								 rospy.Time.now(),
								 "p" + str(i),
								 "object")
			br.sendTransform((v0[0], v0[1], v0[2]),
							 (0, 0, 0, 1),
							 rospy.Time.now(),
							 "object",
							 "base_link")	
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)