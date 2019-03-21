#!/usr/bin/env python
import numpy as np
import sys
import select
import rospy
import tf
import math
import collections

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from ur5_control import ur5_control

debug = False		# Print Debug-Messages

class gtPose():
	def __init__(self, poseBuffSize):
		##################################
		# Give parameters in deg, meters #
		##################################
		self.poseBuffSize = poseBuffSize
		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()
		self.tfBroadcaster = tf.TransformBroadcaster()

		# Init variables
		self.markerPoses = PoseArray()
		self.meanPose = Pose()
		self.poseBuff = collections.deque(maxlen=self.poseBuffSize)

		self.ur5 = ur5_control.ur5Controler("camera_planning_frame", "/mean_marker_pose", True)

		rospy.sleep(1)	# Wait to register at tf
		# Subscriber to Object-Pose
		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.marker_pose_callback, queue_size=1)	# Marker-Pose
		rospy.Subscriber("/fiducial_images", Image, self.image_callback)

		# Publisher for Pose-Array
		self.pub = rospy.Publisher('/tf_meanMarkerPose', Pose, queue_size=10)

	def image_callback(self, data):
		img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Visualization
		cv2.imshow("Img", img)
		cv2.waitKey(1)

	# Subscribe to object pose
	def marker_pose_callback(self, data):
		'''self.markerPose.position.x = data.transforms[0].transform.translation.x
		self.markerPose.position.y = data.transforms[0].transform.translation.y
		self.markerPose.position.z = data.transforms[0].transform.translation.z
		self.markerPose.orientation.x = data.transforms[0].transform.rotation.x
		self.markerPose.orientation.y = data.transforms[0].transform.rotation.y
		self.markerPose.orientation.z = data.transforms[0].transform.rotation.z
		self.markerPose.orientation.w = data.transforms[0].transform.rotation.w'''
		
		# If a new valid pose is available: Store actual pose from tf to ring-buffer
		if len(data.transforms) == 1:
			self.get_pose()

	# Get transformation between base and marker
	def get_pose(self):
		try:
			# Get transformation - which is refined by tf_marker_broadcaster
			(trans, rot) = self.tfListener.lookupTransform('/base_link', '/m_det', rospy.Time(0))
			self.poseBuff.append(self.listToPose(trans, rot))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

	# Return a 6D-Array from given Twist
	def twist_to_array(self, t):
		return [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]

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

	# Store given pose in array
	def store_pose(self, poseToStore):
		self.markerPoses.poses.append(poseToStore)

		# Empty the buffer
		self.poseBuff.clear()

	def empty_buff(self):
		self.poseBuff.clear()

	def ringBuff_to_poseArray(self, ringBuff):
		poseArray = PoseArray()
		for i in range(len(ringBuff)):
			poseArray.poses.append(ringBuff[i])

		return poseArray

	# Caclulate the mean pose of all given poses
	def calc_mean_pose(self, poseArray):
		sumPose = Pose()
		meanPose = Pose()
		sumAngles = [0.0, 0.0, 0.0]
		numPoses = len(poseArray.poses)
		if numPoses == 0:
			return

		# Calculate sum of poses for mean pose
		for i in range(numPoses):
			print_debug("Pose " + str(i))
			print_debug(poseArray.poses[i])
			sumPose.position.x = sumPose.position.x + poseArray.poses[i].position.x
			sumPose.position.y = sumPose.position.y + poseArray.poses[i].position.y
			sumPose.position.z = sumPose.position.z + poseArray.poses[i].position.z
			angles = tf.transformations.euler_from_quaternion([poseArray.poses[i].orientation.x, poseArray.poses[i].orientation.y, poseArray.poses[i].orientation.z, poseArray.poses[i].orientation.w])
			sumAngles = [sumAngles[i] + angles[i] for i in range(len(sumAngles))]

		# Calculate mean pose
		meanPose.position.x = sumPose.position.x / numPoses
		meanPose.position.y = sumPose.position.y / numPoses
		meanPose.position.z = sumPose.position.z / numPoses

		meanAngles = [sumAngles[i] / numPoses for i in range(len(sumAngles))]
		meanOrientations = tf.transformations.quaternion_from_euler(0, 0, meanAngles[2])	# Rotation around x/y is impossible
		meanPose.orientation.x = meanOrientations[0]
		meanPose.orientation.y = meanOrientations[1]
		meanPose.orientation.z = meanOrientations[2]
		meanPose.orientation.w = meanOrientations[3]

		return meanPose

	# Broadcast all recorded gt-poses for comparison
	def broadcast_gtPosesComparison(self, gtPoses):
		br = tf.TransformBroadcaster()
		for i in range(len(gtPoses.poses)):
			br.sendTransform((gtPoses.poses[i].position.x, gtPoses.poses[i].position.y, gtPoses.poses[i].position.z),
							 (gtPoses.poses[i].orientation.x, gtPoses.poses[i].orientation.y, gtPoses.poses[i].orientation.z, gtPoses.poses[i].orientation.w),
							 rospy.Time.now(),
							 "gt_" + str(i),
							 "base_link")

	# Broadcast the final mean pose and other frames dependent on that frame
	def broadcast_transformations(self):
		self.tfBroadcaster.sendTransform((self.meanPose.position.x , self.meanPose.position.y , self.meanPose.position.z ),
						 (self.meanPose.orientation.x, self.meanPose.orientation.y, self.meanPose.orientation.z, self.meanPose.orientation.w),
						 rospy.Time.now(),
						 "mean_marker_pose",
						 "base_link")	# marker-pose wrt to base_link
		self.tfBroadcaster.sendTransform((0, 0, 0.05),
						 (0, 0, 0, 1),
						 rospy.Time.now(),
						 "object_img_center",
						 "mean_marker_pose")  # calculated pose where camera should point to
		self.tfBroadcaster.sendTransform((-0.1185, 0, 0.0625),
						 (tf.transformations.quaternion_from_euler(90*math.pi/180, 90*math.pi/180, 0, "ryzx")),
						 rospy.Time.now(),
						 "object_grasp",
						 "mean_marker_pose")  # calculated pose where the object can be grasped
		self.tfBroadcaster.sendTransform((0, 0, -0.0075),
						 (0, 0, 0, 1),
						 rospy.Time.now(),
						 "object",
						 "mean_marker_pose")  # calculated pose where the object base is (z pointing up)
		self.tfBroadcaster.sendTransform((0, 0, -0.0075),
						 (tf.transformations.quaternion_from_euler(-90*math.pi/180, 0, -180*math.pi/180, "rxyz")),
						 rospy.Time.now(),
						 "object_origin",
						 "mean_marker_pose")  # calculated pose where the object base is (for placement of cad-file)

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init node
	rospy.init_node('gtPose_calc', anonymous=True, disable_signals=True)

	if len(args) < 2:
		buffLen = 10
		rospy.logwarn("Buffersize set to default " + str(buffLen) +". Give integer when running program to change.")
	else:
		buffLen = int(args[1])

	# Initialize Pose-Calculator pC
	pC = gtPose(buffLen)

	'''mean = Pose()
	mean.position.x = 0.2
	mean.position.y = 0.3
	mean.position.z = 0.4
	mean.orientation.x = 0#0.245	# 20, 30, 40
	mean.orientation.y = 0#0.182
	mean.orientation.z = 0#0.368
	mean.orientation.w = 1#0.879

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

	pC.poseBuff.append(mean)
	pC.poseBuff.append(m1)
	pC.poseBuff.append(m2)'''

	# Clear the robot's world
	pC.ur5.scene.remove_world_object()

	# Publish pose to make old pose gone
	rate = rospy.Rate(10)
	for k in range(10):
		pC.pub.publish(pC.meanPose)
		pC.broadcast_gtPosesComparison(pC.markerPoses)
		rate.sleep()

	# Capture poses from different views
	while True:
		inp = raw_input("Press 'y' to store Pose, 'c' to calculate accuracy, 'e' to empty buffer and 'f' if you have finished recording poses. ")[0]
		if inp == 'y':
			while True:
				if len(pC.poseBuff) >= pC.poseBuffSize:
					break
			poseBuffLoc = pC.ringBuff_to_poseArray(pC.poseBuff) 	# Make local copy so it does not change at the moment and convert
			meanPose = pC.calc_mean_pose(poseBuffLoc)				# Calculate mean pose of ring-buffer
			minD, maxD = pC.calc_metrics(meanPose, poseBuffLoc)		# Calculate deviation in ring-buffer
			pC.disp_metrics(minD, maxD)
			inp = raw_input("Press 'y' to store Pose. ")[0]
			if inp == 'y':
				pC.store_pose(meanPose)								# Store pose to array
				print "Pose stored"

		elif inp == 'e':
			pC.empty_buff()

		elif inp == 'c':
			poseBuffLoc = pC.ringBuff_to_poseArray(pC.poseBuff) 	# Make local copy so it does not change at the moment and convert
			meanPose = pC.calc_mean_pose(poseBuffLoc)				# Calculate mean pose of ring-buffer
			minD, maxD = pC.calc_metrics(meanPose, poseBuffLoc)		# Calculate deviation in ring-buffer
			pC.disp_metrics(minD, maxD)

		elif inp == 'f':
			# Calculate and print mean pose
			print "-------------- Mean Object Pose --------------"
			pC.meanPose = pC.calc_mean_pose(pC.markerPoses)
			print pC.meanPose
			minD, maxD = pC.calc_metrics(pC.meanPose, pC.markerPoses)
			pC.disp_metrics(minD, maxD)
			pC.broadcast_transformations()
			rospy.sleep(1)
			pC.ur5.addMesh(Pose())	# TODO change to be correct
			break

	print "Publishing ground-truth Object-Pose..."
	print "Store pose by typing 'rosbag record /tf_meanMarkerPose' and replay them by typing 'rosbag play -l'."


	# Broadcast transformations
	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		pC.pub.publish(pC.meanPose)
		pC.broadcast_gtPosesComparison(pC.markerPoses)	# Comparison of stored poses
		pC.broadcast_transformations()
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)

#sys.stdout.write("\033[F") # Cursor up one line
#sys.stdout.write("\033[K") # Clear to the end of line