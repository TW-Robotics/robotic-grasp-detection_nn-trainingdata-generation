#!/usr/bin/env python  
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge, CvBridgeError

debug = True

class pose_broadcaster():
	def __init__(self):
		self.bridge = CvBridge()
		self.br = tf.TransformBroadcaster()
		self.tfListener = tf.TransformListener()
		self.objectPoseCarrier = None
		self.objectPoseHolder = None
		self.resetPose_carrier = False
		self.resetPose_holder = False
		self.thr = 0.1 # Threshold to use pose for calculation of mean pose

		# Init subscriber
		rospy.Subscriber("/dope/pose_carrier_empty", PoseStamped, self.carrier_pose_callback, queue_size=1)				# Pose transform camera to carrier-object
		rospy.Subscriber("/dope/pose_holder_empty", PoseStamped, self.holder_pose_callback, queue_size=1)				# Pose transform camera to holder-object
		rospy.Subscriber("/dope/rgb_points", Image, self.rgb_image_callback)									# RGB-Image
		rospy.Subscriber("/dope/has_grasped", Bool, self.has_grasped_callback)
		rospy.Subscriber("/dope/has_put", Bool, self.has_put_callback)
		self.carrier_pose_update_pub = rospy.Publisher("/dope/pose_update_carrier", Bool, queue_size=10)
		self.holder_pose_update_pub = rospy.Publisher("/dope/pose_update_holder", Bool, queue_size=10)

	def has_grasped_callback(self, data):
		self.resetPose_carrier = True

	def has_put_callback(self, data):
		self.resetPose_holder = True

	def rgb_image_callback(self, data):
		rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Visualization
		cv2.imshow("Dope output", rgb_img)
		cv2.waitKey(1)

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
		meanOrientations = tf.transformations.quaternion_from_euler(meanAngles[0], meanAngles[1], meanAngles[2])
		meanPose.orientation.x = meanOrientations[0]
		meanPose.orientation.y = meanOrientations[1]
		meanPose.orientation.z = meanOrientations[2]
		meanPose.orientation.w = meanOrientations[3]
		return meanPose

	def check_deviation(self, ref, pose):
		if abs(pose.position.x - ref.position.x) <= self.thr:
			if abs(pose.position.y - ref.position.y) <= self.thr:
				if abs(pose.position.z - ref.position.z) <= self.thr:
					return True
		print "Pose deviation too big!"
		print ref, pose
		return False

	def holder_pose_callback(self, objectPose):
		newPose = self.send_and_listen(objectPose, "holder")
		self.objectPoseHolder = self.mean_and_publish(self.objectPoseHolder, newPose)
		# Inform other nodes that object pose has been updated
		self.holder_pose_update_pub.publish(Bool(True))

	def carrier_pose_callback(self, objectPose):
		newPose = self.send_and_listen(objectPose, "carrier")
		self.objectPoseCarrier = self.mean_and_publish(self.objectPoseCarrier, newPose)
		# Inform other nodes that object pose has been updated
		self.carrier_pose_update_pub.publish(Bool(True))

	# Broadcast object-pose w.r.t camera to tf and then lookup transform object-pose w.r.t. base_link
	def send_and_listen(self, objectPose, objectType):
		frameName = "dope_object_pose_" + objectType
		now = rospy.Time.now()
		# Broadcast camera -> object
		self.br.sendTransform((objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z),
							 (objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z, objectPose.pose.orientation.w),
							 now,
							 frameName,
							 "camera_color_optical_frame")

		rospy.sleep(0.1)
		# Lookup and store base -> object
		try:
			(trans, rot) = self.tfListener.lookupTransform('/base_link', frameName, now)
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)
			return None

	def mean_and_publish(self, objectPose, newObjectPose):
		if objectPose is None:
			objectPose = newObjectPose
		elif self.check_deviation(objectPose, newObjectPose) == True:
			poseArray = PoseArray()
			poseArray.poses.append(objectPose)
			poseArray.poses.append(newObjectPose)
			objectPose = self.calc_mean_pose(poseArray) 	# Calculate mean pose of pose up to now and new pose 
			print "Mean pose calculated."

		return objectPose

	# Broadcast mean obejct-pose w.r.t. base (So object does not move if camera moves)
	def broadcast_pose(self, objectPose, objName):
		self.br.sendTransform((objectPose.position.x, objectPose.position.y, objectPose.position.z),
					 (objectPose.orientation.x, objectPose.orientation.y, objectPose.orientation.z, objectPose.orientation.w),
					 rospy.Time.now(),
					 "dope_object_pose_" + objName,
					 "base_link")

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init Node
	rospy.init_node('tf_dope_pose_broadcaster', disable_signals=True)

	pose_br = pose_broadcaster()

	rospy.loginfo("TF-Grasp-Point-Broadcaster sucessfully launched!\nBroadcasting to tf...")

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if pose_br.resetPose_carrier == True:
			print "Deleting carrier-pose from TF"
			pose_br.objectPoseCarrier = None
			pose_br.resetPose_carrier = False
		if pose_br.resetPose_holder == True:
			print "Deleting holder-pose from TF"
			pose_br.objectPoseHolder = None
			pose_br.resetPose_holder = False

		if pose_br.objectPoseCarrier is not None:
			pose_br.broadcast_pose(pose_br.objectPoseCarrier, "carrier")
		if pose_br.objectPoseHolder is not None:
			pose_br.broadcast_pose(pose_br.objectPoseHolder, "holder")

		rate.sleep()

	#rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
