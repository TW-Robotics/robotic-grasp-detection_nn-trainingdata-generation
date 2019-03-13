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
		rospy.init_node('gtPose_calc', anonymous=True, disable_signals=True)

		# Subscriber to Object-Pose
		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.marker_pose_callback, queue_size=1)

		# Publisher for Pose-Array
		self.pub = rospy.Publisher('/gtPoses', PoseArray, queue_size=10)

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
		if len(data.transforms) == 1:
			self.poseError = data.transforms[0].object_error
		#self.poseError = data.transforms[0].object_error

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
			(trans, rot) = self.tfListener.lookupTransform('/base_link', '/marker_245', rospy.Time(0))
			self.poses.poses.append(self.listToPose(trans, rot))
			self.poseErrors.append(self.poseError)
			print self.listToPose(trans, rot)
			print self.poseError
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("Warning!")

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	poseCalculator = gtPose()
	i = 0
	rate = rospy.Rate(10)
	for k in range(10):
		poseCalculator.pub.publish(poseCalculator.poses)
		rate.sleep()
	while True:
		inp = raw_input("Store position? y/n: ")[0]
		if inp == 'y':
			poseCalculator.get_pose(i)
			print "Pose stored"
			i = i + 1
		elif inp == 'n':
			print "-------------- DONE --------------"
			sumPose = Pose()
			numPoses = len(poseCalculator.poses.poses)
			for i in range(numPoses):
				print "Pose " + str(i) + " - Error:" + str(poseCalculator.poseErrors[i])
				print poseCalculator.poses.poses[i]
				sumPose.position.x = sumPose.position.x + poseCalculator.poses.poses[i].position.x
				sumPose.position.y = sumPose.position.y + poseCalculator.poses.poses[i].position.y
				sumPose.position.z = sumPose.position.z + poseCalculator.poses.poses[i].position.z
				sumPose.orientation.x = sumPose.orientation.x + poseCalculator.poses.poses[i].orientation.x
				sumPose.orientation.y = sumPose.orientation.y + poseCalculator.poses.poses[i].orientation.y
				sumPose.orientation.z = sumPose.orientation.z + poseCalculator.poses.poses[i].orientation.z
				sumPose.orientation.w = sumPose.orientation.w + poseCalculator.poses.poses[i].orientation.w
			poseCalculator.pub.publish(poseCalculator.poses)
			break
	'''meanPose = Pose()
	meanPose.position.x = sumPose.position.x / numPoses
	meanPose.position.y = sumPose.position.y / numPoses
	meanPose.position.z = sumPose.position.z / numPoses
	meanPose.orientation.x = sumPose.orientation.x / numPoses
	meanPose.orientation.y = sumPose.orientation.y / numPoses
	meanPose.orientation.z = sumPose.orientation.z / numPoses
	meanPose.orientation.w = sumPose.orientation.w / numPoses
	while True:
		print meanPose
		poseCalculator.pub.publish(meanPose)'''

if __name__ == '__main__':
	main(sys.argv)