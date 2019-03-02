#!/usr/bin/env python
import numpy as np
import sys
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d

import rospy
import tf
from geometry_msgs.msg import Pose
import math

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

	px = 0.5
	py = 0.5
	pz = 0.5

	ox = 0
	oy = 0
	oz = 0

	vx = ox - px
	vy = oy - py
	vz = oz - pz

	#print vx, vy, vz

	v1 = np.array([0.5, 0.5, 0.5])
	v2 = np.array([1., 0, 0])

	a = np.cross(v1, v2)
	print a
	l1 = np.linalg.norm(v1, ord=2)
	l2 = np.linalg.norm(v2, ord=2)
	w = math.sqrt(l1*l2) + v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
	print w

	#q = tf.normalized(a[0], a[1], a[2], w)
	#print q
	q = np.array([0, 0.3134257, -0.3134257, 0.8963976])

	'''vector a = crossproduct(v1, v2);
	q.xyz = a
	q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2)'''

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
		br.sendTransform((px, py, pz),
						 (q[0], q[1], q[2], q[3]),
						 rospy.Time.now(),
						 "p" + str(i),
						 "object")
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)