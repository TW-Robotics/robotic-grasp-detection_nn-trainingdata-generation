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
	vi = np.array([0.5, 0.8, 0.7])
	
	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		for i in range(5):#len(pointsx)):
			vi = [-pointsx[i], -pointsy[i], -pointsz[i]]
			v10 = vi

			xyLength = math.sqrt(v10[0]*v10[0] + v10[1]*v10[1])
			vecLength = math.sqrt(v10[0]*v10[0] + v10[1]*v10[1] + v10[2]*v10[2])
			print xyLength, vecLength
			
			zAngle = math.acos(v10[1] / xyLength)
			xAngle = math.acos(xyLength / vecLength)

			if v10[2] < 0:
				#print "xCorr"
				xAngle = -xAngle
			if v10[0] > 0:
				#print "zCorr"
				zAngle = -zAngle

			print xAngle*180/math.pi, zAngle*180/math.pi

			q = tf.transformations.quaternion_from_euler(xAngle-math.pi/2, 0, zAngle)
			print q

			'''br.sendTransform((pointsx[i], pointsy[i], pointsz[i]),
								 (0, 0, 0, 1),
								 rospy.Time.now(),
								 "p" + str(i),
								 "object")'''
			br.sendTransform((pointsx[i], pointsy[i], pointsz[i]),
							 (q[0], q[1], q[2], q[3]),
							 rospy.Time.now(),
							 "p" + str(i),
							 "base_link")
			'''br.sendTransform((v1[0], v1[1], v1[2]),
							 (q[0], q[1], q[2], q[3]),
							 rospy.Time.now(),
							 "p1",
							 "base_link")
			br.sendTransform((v0[0], v0[1], v0[2]),
							 (0, 0, 0, 1),
							 rospy.Time.now(),
							 "p0",
							 "base_link")'''		
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)