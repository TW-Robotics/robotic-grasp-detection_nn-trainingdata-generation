#!/usr/bin/env python
import numpy as np
import sys

import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math
from geometry_msgs.msg import Quaternion
from ur5_control import ur5_control
from fiducial_msgs.msg import FiducialTransformArray

debug = False		# Print Debug-Messages

class gtPose():
	def __init__(self):
		# Init node
		rospy.init_node('capture_pose_calc', anonymous=True, disable_signals=True)

		# Subscriber to Object-Pose
		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.marker_pose_callback, queue_size=1)

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()

		# Init variables
		self.markerPose = Pose()
		self.poseError = 0
		self.poses = PoseArray()
		self.poseErrors = []

		##################################
		# Give parameters in deg, meters #
		##################################

		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

	# Subscribe to object pose
	def marker_pose_callback(self, data):
		'''self.markerPose.position.x = data.transforms[0].transform.translation.x
		self.markerPose.position.y = data.transforms[0].transform.translation.y
		self.markerPose.position.z = data.transforms[0].transform.translation.z
		self.markerPose.orientation.x = data.transforms[0].transform.rotation.x
		self.markerPose.orientation.y = data.transforms[0].transform.rotation.y
		self.markerPose.orientation.z = data.transforms[0].transform.rotation.z
		self.markerPose.orientation.w = data.transforms[0].transform.rotation.w'''
		self.poseError = data.transforms[0].object_error

	'''def copy_act_pose(self):
		pose = Pose()
		pose.position.x = self.markerPose.position.x
		pose.position.y = self.markerPose.position.y
		pose.position.z = self.markerPose.position.z
		pose.orientation.x = self.markerPose.orientation.x
		pose.orientation.y = self.markerPose.orientation.y
		pose.orientation.z = self.markerPose.orientation.z
		pose.orientation.w = self.markerPose.orientation.w

		poseError = self.poseError'''

	# Convert lists to pose-obect so a standard pose message can be published
	def listToPose(self, trans, rot):
		pose = Pose()
		pose.position.x = trans[0]
		pose.position.y = trans[1]
		pose.position.z = trans[2]
		pose.orientation.x = rot[0]
		pose.orientation.y = rot[1]
		pose.orientation.z = rot[2]
		pose.orientation.w = rot[3]
		return pose

	def get_pose(self, id):
		try:
			# Get transformation
			(trans, rot) = self.tfListener.lookupTransform('/marker_240', '/base_link', rospy.Time(0))
			self.poses[id] = self.listToPose(trans, rot)
			self.poseErrors[id] = self.poseError
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("Warning!")

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	poseCalculator = gtPose()
	for i in range(3):
		inp = raw_input("Store position? y/n: ")[0]
		if inp == 'y':
			poseCalculator.get_pose(i)
			print "Pose stored"
	for i in range(len(poseCalculator.poses)):
		print "Pose 1 - Error:" + str(poseCalculator.poseErrors[i])
		print poseCalculator.poses[i]

if __name__ == '__main__':
	main(sys.argv)