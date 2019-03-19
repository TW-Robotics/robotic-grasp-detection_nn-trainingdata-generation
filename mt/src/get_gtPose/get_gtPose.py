#!/usr/bin/env python
import numpy as np
import sys
import select
import rospy
import tf
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

debug = True		# Print Debug-Messages

class gtPose():
	def __init__(self):
		# Init node
		rospy.init_node('gtPose_calc', anonymous=True, disable_signals=True)

		##################################
		# Give parameters in deg, meters #
		##################################
		self.marker_id = "\marker_245"
		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()
		self.tfBroadcaster = tf.TransformBroadcaster()

		# Init variables
		self.poseError = 0
		self.markerPoses = PoseArray()
		self.poseErrors = []
		self.dImage = Image()
		self.markerCenter = Point()
		self.meanPose = Pose()

		# Subscriber to Object-Pose
		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.marker_pose_callback, queue_size=1)	# Marker-Pose
		rospy.Subscriber("/fiducial_vertices", FiducialArray, self.marker_vert_callback, queue_size=1)				# Marker-Corners
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)									# RGB-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.d_image_callback)					# Depth-Image

		# Publisher for Pose-Array
		self.pub = rospy.Publisher('/gtPoses', PoseArray, queue_size=10)
		self.pubDepth = rospy.Publisher('/markerCenter', Point, queue_size=10)

	def rgb_image_callback(self, data):
		rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		''' # Visualization
		cv2.circle(rgb_img ,(int(self.markerCenter.x), int(self.markerCenter.y)),2,(0,0,255),3)
		cv2.imshow("Img", rgb_img)
		cv2.waitKey(1)'''

	def d_image_callback(self, data):
		self.d_img = self.bridge.imgmsg_to_cv2(data, "16UC1")

		# If a depth-image arrives, calculate the depth-value of the marker's center and publish it
		self.markerCenter.z = self.d_img[int(self.markerCenter.y)][int(self.markerCenter.x)]
		self.pubDepth.publish(self.markerCenter)

	def marker_vert_callback(self, data):
		# If at least one marker could be found, calculate its center
		if len(data.fiducials) > 0:
			vert = data.fiducials[0]
			self.markerCenter.x = int((vert.x0+vert.x1+vert.x2+vert.x3)/4)
			self.markerCenter.y = int((vert.y0+vert.y1+vert.y2+vert.y3)/4)

	# Subscribe to object pose
	def marker_pose_callback(self, data):
		'''self.markerPose.position.x = data.transforms[0].transform.translation.x
		self.markerPose.position.y = data.transforms[0].transform.translation.y
		self.markerPose.position.z = data.transforms[0].transform.translation.z
		self.markerPose.orientation.x = data.transforms[0].transform.rotation.x
		self.markerPose.orientation.y = data.transforms[0].transform.rotation.y
		self.markerPose.orientation.z = data.transforms[0].transform.rotation.z
		self.markerPose.orientation.w = data.transforms[0].transform.rotation.w'''
		# Store actual pose-error
		if len(data.transforms) == 1:
			self.poseError = data.transforms[0].object_error

	# Calculate deviation of p2 from p1 (p1 = source = "ground truth")
	def calc_deviation(self, p1, p2):
		poseDeviation = Twist()

		# Get euler angles from quaternions
		angles1 = tf.transformations.euler_from_quaternion([p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w])
		angles2 = tf.transformations.euler_from_quaternion([p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w])

		# Calculate deviations
		poseDeviation.linear.x = p2.position.x - p1.position.x
		poseDeviation.linear.y = p2.position.y - p1.position.y
		poseDeviation.linear.z = p2.position.z - p1.position.z
		poseDeviation.angular.x = angles2[0] - angles1[0]
		poseDeviation.angular.y = angles2[1] - angles1[1]
		poseDeviation.angular.z = angles2[2] - angles1[2]
		return poseDeviation

	# Return a 6D-Array from given Twist
	def twist_to_array(self, t):
		return [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]

	# Calculate the biggest deviation of all poses in p from reference ref
	def calc_metrics(self, ref, p):
		minDev = [0., 0., 0., 0., 0., 0.]
		maxDev = [0., 0., 0., 0., 0., 0.]

		for i in range(len(p.poses)):
			dev = self.twist_to_array(self.calc_deviation(ref, p.poses[i]))
			for j in range(len(dev)):
				if dev[j] < minDev[j]:
					minDev[j] = dev[j]
				if dev[j] > maxDev[j]:
					maxDev[j] = dev[j]

		return minDev, maxDev

	# Display metrics information
	def disp_metrics(self, minD, maxD):
		print "Deviation:"
		for i in range(0, 3):
			print int(round(minD[i]*1000, 0)), int(round(maxD[i]*1000, 0))
		for i in range(3, 6):
			print round(minD[i]* 180/math.pi, 1), round(maxD[i]* 180/math.pi, 1)

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

	# Get transformation between base and marker
	def get_pose(self, id):
		try:
			# Get transformation
			(trans, rot) = self.tfListener.lookupTransform('/base_link', self.marker_id, rospy.Time(0))
			self.markerPoses.poses.append(self.listToPose(trans, rot))
			self.poseErrors.append(self.poseError)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

	# Caclulate the mean pose of all given poses
	def calc_mean_pose(self):
		sumPose = Pose()
		sumAngles = [0.0, 0.0, 0.0]
		numPoses = len(self.markerPoses.poses)

		# Calculate sum of poses for mean pose
		for i in range(numPoses):
			print_debug("Pose " + str(i) + " - Error:" + str(self.poseErrors[i]))
			print_debug(self.markerPoses.poses[i])
			sumPose.position.x = sumPose.position.x + self.markerPoses.poses[i].position.x
			sumPose.position.y = sumPose.position.y + self.markerPoses.poses[i].position.y
			sumPose.position.z = sumPose.position.z + self.markerPoses.poses[i].position.z
			angles = tf.transformations.euler_from_quaternion([self.markerPoses.poses[i].orientation.x, self.markerPoses.poses[i].orientation.y, self.markerPoses.poses[i].orientation.z, self.markerPoses.poses[i].orientation.w])
			sumAngles = [sumAngles[i] + angles[i] for i in range(len(sumAngles))]

		# Calculate mean pose
		self.meanPose.position.x = sumPose.position.x / numPoses
		self.meanPose.position.y = sumPose.position.y / numPoses
		self.meanPose.position.z = sumPose.position.z / numPoses

		meanAngles = [sumAngles[i] / numPoses for i in range(len(sumAngles))]
		meanOrientations = tf.transformations.quaternion_from_euler(meanAngles[0], meanAngles[1], meanAngles[2])
		self.meanPose.orientation.x = meanOrientations[0]
		self.meanPose.orientation.y = meanOrientations[1]
		self.meanPose.orientation.z = meanOrientations[2]
		self.meanPose.orientation.w = meanOrientations[3]

def broadcast_transformations(self):
	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		pC.tfBroadcaster.sendTransform((self.meanPose.position.x , self.meanPose.position.y , self.meanPose.position.z ),
						 (self.meanPose.orientation.x, self.meanPose.orientation.y, self.meanPose.orientation.z, self.meanPose.orientation.w),
						 rospy.Time.now(),
						 "mean_marker_pose",
						 "base_link")	# marker-pose wrt to base_link
		pC.tfBroadcaster.sendTransform((0, 0, 0.05),
						 (0, 0, 0, 1),
						 rospy.Time.now(),
						 "object_img_center",
						 "mean_marker_pose")  # calculated pose where camera should point to
		pC.tfBroadcaster.sendTransform((-0.1, 0, 0.08),
						 (tf.transformations.quaternion_from_euler(90*math.pi/180, 90*math.pi/180, 0, "ryzx")),
						 rospy.Time.now(),
						 "object",
						 "mean_marker_pose")  # calculated pose where the object can be grasped
		rate.sleep()

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Initialize Pose-Calculator pC
	pC = gtPose()

	mean = Pose()
	mean.position.x = 2
	mean.position.y = 3
	mean.position.z = 4
	mean.orientation.x = 0.245	# 20, 30, 40
	mean.orientation.y = 0.182
	mean.orientation.z = 0.368
	mean.orientation.w = 0.879

	markers = PoseArray()
	m1 = Pose()
	m2 = Pose()
	m1.position.x = 3
	m1.position.y = 4
	m1.position.z = 2
	m1.orientation.x = 0.271	# 25, 25, 40
	m1.orientation.y = 0.126
	m1.orientation.z = 0.370
	m1.orientation.w = 0.880

	m2.position.x = -1
	m2.position.y = 3
	m2.position.z = 1
	m2.orientation.x = 0.212	# 18, 20, 50
	m2.orientation.y = 0.090
	m2.orientation.z = 0.436
	m2.orientation.w = 0.870

	markers.poses.append(m2)
	markers.poses.append(m1)

	# Publish pose to make old pose gone
	rate = rospy.Rate(10)
	for k in range(10):
		pC.pub.publish(pC.markerPoses)
		rate.sleep()

	# Capture poses from different views
	i = 0
	while True:
		inp = raw_input("Press 'y' to store Pose, 'c' to calculate accuracy and 'd' if you have finished recording poses. ")[0]
		if inp == 'y':
			minD, maxD = pC.calc_metrics(mean, markers)
			pC.disp_metrics(minD, maxD)
			inp = raw_input("Press 'y' to store Pose. ")[0]
			if inp == 'y':
				pC.get_pose(i)
				print "Pose stored"
				i = i + 1
		elif inp == 'c':
			minD, maxD = pC.calc_metrics(mean, markers)
			pC.disp_metrics(minD, maxD)
		elif inp == 'd':
			# Publish poses for comparison
			rate = rospy.Rate(10)
			for k in range(10):
				pC.pub.publish(pC.markerPoses)
				rate.sleep()

			# Calculate and print mean pose
			print "-------------- Mean Object Pose --------------"
			pC.calc_mean_pose()
			print pC.meanPose
			minD, maxD = pc.calc_metrics(pC.meanPose, pC.markerPoses)
			pC.disp_metrics(minD, maxD)
			break

	# Broadcast transformations
	pC.broadcast_transformations()

if __name__ == '__main__':
	main(sys.argv)

#sys.stdout.write("\033[F") # Cursor up one line
#sys.stdout.write("\033[K") # Clear to the end of line