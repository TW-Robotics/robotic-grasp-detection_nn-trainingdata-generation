#!/usr/bin/env python
import numpy as np
import sys
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d

import rospy
import tf
from geometry_msgs.msg import Pose
import math
from geometry_msgs.msg import Quaternion

def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    return vec

def main(args):
	rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

	phi = np.linspace(0, np.pi, 20)
	theta = np.linspace(0, 2 * np.pi, 40)
	r = 0.7
	x = np.outer(np.sin(theta), np.cos(phi)) * r
	y = np.outer(np.sin(theta), np.sin(phi)) * r
	z = np.outer(np.cos(theta), np.ones_like(phi)) * r

	pointsx = []
	pointsy = []
	pointsz = []

	xi, yi, zi = sample_spherical(100) * r
	for i in range(len(xi)):
		if zi[i] > (-r/2.5):
			pointsx.append(xi[i])
			pointsy.append(yi[i])
			pointsz.append(zi[i])

	#fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d', 'aspect':'equal'})
	#ax.plot_wireframe(x, y, z, color='k', rstride=1, cstride=1)
	#ax.scatter(pointsx, pointsy, pointsz, s=100, c='r', zorder=10)
	#plt.show()

	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()

	# From v1 to v0 pointing
	v0 = np.array([0, 0, 0])
	v1 = np.array([-0.02372714335579641, 0.15962260421743776, 0.6811443656751603])
	v1 = np.array([1, 1, 1])

	v10 = -v1
	print v10

	xyLength = math.sqrt(v10[0]*v10[0] + v10[1]*v10[1])
	vecLength = math.sqrt(v10[0]*v10[0] + v10[1]*v10[1] + v10[2]*v10[2])
	print xyLength, vecLength
	if xyLength == 0:
		if v10[0] > 0:
			zAngle = 0#math.pi/2
		else:
			zAngle = 0#-math.pi/2
	else:
		zAngle = math.acos(v10[1] / xyLength)
	
	xAngle = math.acos(xyLength / vecLength)
	if v10[0] > 0:
		print "zCorr"
		zAngle = -zAngle
	if (v10[2] > 0 and v10[1] < 0) or (v10[2] < 0 and v10[1] > 0):
		print "xCorr"
		xAngle = -xAngle

	print xAngle*180/math.pi, zAngle*180/math.pi

	q = tf.transformations.quaternion_from_euler(xAngle, 0, zAngle, 'rxyz')
	#q1 = tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0, 'rxyz')
	#q = tf.transformations.quaternion_multiply(q, q1)

	'''q = [0., 0, 0, 0]
	angle = math.atan2( v10[2], v10[0] )
	q[0] = v0[0] * math.sin( angle/2)
	q[1] = v0[1] * math.sin( angle/2)
	q[2] = v0[2] * math.sin( angle/2)
	q[3] = math.cos( angle/2 )

	qNorm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
	q = [q[0]/qNorm, q[1]/qNorm, q[2]/qNorm, q[3]/qNorm]'''

	# Calculate Quaternions
	'''a = np.cross(v1, v10)
	w = math.sqrt(np.linalg.norm(v1, ord=2) * np.linalg.norm(v10, ord=2)) + v1[0]*v10[0] + v1[1]*v10[1] + v1[2]*v10[2]
	# Normalize Quaternions
	qNorm = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + w*w)
	#q = [a[0]/qNorm, a[1]/qNorm, a[2]/qNorm, w/qNorm]'''
	#print q

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Send transformation to tf
		'''for i in range(len(pointsx)):
			br.sendTransform((pointsx[i], pointsy[i], pointsz[i]),
							 (0, 0, 0, 1),
							 rospy.Time.now(),
							 "p" + str(i),
							 "object")'''
		br.sendTransform((v1[0], v1[1], v1[2]),
						 (q[0], q[1], q[2], q[3]),
						 rospy.Time.now(),
						 "p1",
						 "p0")
		br.sendTransform((v0[0], v0[1], v0[2]),
						 (0, 0, 0, 1),
						 rospy.Time.now(),
						 "p0",
						 "object")		
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)