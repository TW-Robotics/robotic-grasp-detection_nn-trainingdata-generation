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
		self.br = tf.TransformBroadcaster()

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
			#print self.listToPose(trans, rot)
			#print self.poseError
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

	# Publish pose to make old pose gone
	rate = rospy.Rate(10)
	for k in range(10):
		poseCalculator.pub.publish(poseCalculator.poses)
		rate.sleep()

	# Capture views
	while True:
		inp = raw_input("Press 'y' to store Pose and 'n' if you have finished recording poses.")[0]
		if inp == 'y':
			poseCalculator.get_pose(i)
			print "Pose stored"
			i = i + 1
		elif inp == 'n':
			print "-------------- DONE --------------"
			sumPose = Pose()
			sumAngles = [0.0, 0.0, 0.0]
			numPoses = len(poseCalculator.poses.poses)
			for i in range(numPoses):
				print "Pose " + str(i) + " - Error:" + str(poseCalculator.poseErrors[i])
				print poseCalculator.poses.poses[i]
				sumPose.position.x = sumPose.position.x + poseCalculator.poses.poses[i].position.x
				sumPose.position.y = sumPose.position.y + poseCalculator.poses.poses[i].position.y
				sumPose.position.z = sumPose.position.z + poseCalculator.poses.poses[i].position.z
				angles = tf.transformations.euler_from_quaternion([poseCalculator.poses.poses[i].orientation.x, poseCalculator.poses.poses[i].orientation.y, poseCalculator.poses.poses[i].orientation.z, poseCalculator.poses.poses[i].orientation.w])
				sumAngles = [sumAngles[i] + angles[i] for i in range(len(sumAngles))]
				#print angles
			poseCalculator.pub.publish(poseCalculator.poses)	# publish poses for comparison

			# Calculate mean pose
			meanPose = Pose()
			meanPose.position.x = sumPose.position.x / numPoses
			meanPose.position.y = sumPose.position.y / numPoses
			meanPose.position.z = sumPose.position.z / numPoses
			
			meanAngles = [sumAngles[i] / numPoses for i in range(len(sumAngles))]
			meanOrientations = tf.transformations.quaternion_from_euler(meanAngles[0], meanAngles[1], meanAngles[2])
			meanPose.orientation.x = meanOrientations[0]
			meanPose.orientation.y = meanOrientations[1]
			meanPose.orientation.z = meanOrientations[2]
			meanPose.orientation.w = meanOrientations[3]

			print "MEAN OBJECT POSE"
			print meanPose
			break
	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	i = 1
	while not rospy.is_shutdown():
		#try:
			poseCalculator.br.sendTransform((meanPose.position.x , meanPose.position.y , meanPose.position.z ),
							 (meanPose.orientation.x, meanPose.orientation.y, meanPose.orientation.z, meanPose.orientation.w),
							 rospy.Time.now(),
							 "mean_marker_pose",
							 "base_link")
			poseCalculator.br.sendTransform((0, 0, 0.05),
							 (0, 0, 0, 1),
							 rospy.Time.now(),
							 "object_img_center",
							 "mean_marker_pose")
			poseCalculator.br.sendTransform((-0.1, 0, 0.08),
							 (tf.transformations.quaternion_from_euler(90*math.pi/180, 90*math.pi/180, 0, "ryzx")),
							 rospy.Time.now(),
							 "object",
							 "mean_marker_pose")
		#except:
		#	rospy.loginfo("Warning!")
	#		continue
			rate.sleep()

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