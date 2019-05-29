#!/usr/bin/env python
import numpy as np
import sys
import select
import rospy
import tf
import math
import collections
import csv

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge, CvBridgeError

from ur5_control import ur5_control
from gripper_control import gripper_control

debug = False
if rospy.get_param("print_debug") == True:
	print "Debug-Mode ON"
	debug = True		# Print Debug-Messages

class grasp_process():
	def __init__(self):
		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()
		self.tfBroadcaster = tf.TransformBroadcaster()

		self.ur5 = ur5_control.ur5Controler("gripper", "/base_link", True)
		self.gripper = gripper_control.gripper()

		self.graspPose = Pose()
		self.rgb_img = Image()
		self.poseIsUpdated = False

		self.imgOutputSize = rospy.get_param("outputImage_size")
		self.imgWidth = rospy.get_param("camera_width")
		self.imgHeight = rospy.get_param("camera_height")
		self.img_scaleFac = float(self.imgHeight)/self.imgOutputSize
		self.resized_imgWidth = int(round(float(self.imgWidth)/self.img_scaleFac, 0))
		self.resized_img_horizontalStart = int(round(self.resized_imgWidth/2., 0)) - self.imgOutputSize/2

		# Subscriber to pose update
		rospy.Subscriber("/dope/pose_update", Bool, self.pose_update)
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)					# RGB-Image
		self.rawImgPub = rospy.Publisher("/dope/camera_images", Image, queue_size=10)		

		rospy.sleep(1)	# Wait to register at tf

	#def rgb_image_callback(self, data):
	#	self.rgb_img = data

	def rgb_image_callback(self, data):
		try:
			cv_rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			# Store image locally and resize it
			self.rgb_img = cv_rgb_image.copy()
			self.rgb_resize()
		except CvBridgeError as e:
			print(e)

	# Resize the RGB-image to be a square image of output-size
	def rgb_resize(self):
		# Copy image
		rgb_img_resized = self.rgb_img.copy()
		# Change scale so 720px become 400px
		rgb_img_resized = cv2.resize(rgb_img_resized, (self.resized_imgWidth, self.imgOutputSize), interpolation=cv2.INTER_AREA)
		# Cut off pixels at left and right to make image squared
		self.rgb_img_resized = rgb_img_resized[0:self.imgOutputSize, self.resized_img_horizontalStart:self.resized_img_horizontalStart+self.imgOutputSize]
		#print len(img), len(img[0])
		#output = self.rgb_img_resized
		#cv2.imshow("Image-Stream", self.rgb_img_resized)
		#cv2.waitKey(1)

	# Get the last grasp-pose, if a new pose has been sent to tf
	def pose_update(self, data):
		self.update_grasp_pose()

	# Get transformation between base and grasp-point
	def update_grasp_pose(self):
		try:
			(trans, rot) = self.tfListener.lookupTransform('/base_link', '/dope_grasp_point', rospy.Time(0))
			self.graspPose =self.listToPose(trans, rot)
			self.poseIsUpdated = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

	def publish_image(self):
		self.poseIsUpdated = False
		#self.rawImgPub.publish(self.rgb_img)
		self.rawImgPub.publish(self.bridge.cv2_to_imgmsg(self.rgb_img_resized, "bgr8"))

	def calcPrePose(self):
		prePose = self.copyPose(self.graspPose)
		prePose.position.z = prePose.position.z - 0.08
		return prePose

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

	def copyPose(self, pose):
		newPose = Pose()
		newPose.position.x = pose.position.x
		newPose.position.y = pose.position.y
		newPose.position.z = pose.position.z
		newPose.orientation.x = pose.orientation.x
		newPose.orientation.y = pose.orientation.y
		newPose.orientation.z = pose.orientation.z
		newPose.orientation.w = pose.orientation.w
		return newPose

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init node
	rospy.init_node('grasp_object', anonymous=True, disable_signals=True)

	grasper = grasp_process() # TODO Dont forget to update config_pose camera intrinsics

	#grasper.ur5.moveToSearchPose("left")
	while (True):
		inp = raw_input("p to publish image, g to drive to grasp pose: ")[0]
		if inp == 'p':
			grasper.publish_image()
		elif inp == 'g':
			#if grasper.poseIsUpdated == True:
				#print grasper.graspPose
				prePose = grasper.calcPrePose()

		  		pose = Pose()
				pose.position.x = 0.6
				pose.position.y = 0.1
				pose.position.z = 0.37
				pose.orientation.x = 0.#-0.4778
				pose.orientation.y = 0.707#0.4731
				pose.orientation.z = 0#-0.5263
				pose.orientation.w = 0.707#0.5203
				#grasper.ur5.move_to_pose(prePose)
				goalPose = [0.6, 0.1, 0.37, 0, 0, 0.707, 0.707]

				grasper.ur5.move_to_pose(grasper.graspPose)
			#else:
			#	print "No update"
		#grasper.publish_image()
		#rospy.sleep(0.5)
	if grasper.poseIsUpdated == True:
		return True
		prePose = grasper.calcPrePose()
		grasper.ur5.move_to_pose(prePose)
		grasper.ur5.move_to_pose(grasper.graspPose)
		rospy.sleep(5)

		##### Close the gripper to grasp object
		grasper.gripper.close()
		rospy.sleep(5)
		if grasper.gripper.hasGripped() == True:
			print "Successfully grasped object!"
		else:
			print "Error grasping object!"
			return False

		##### Move the robot up and to transport-pose
		ur5.move_xyz(0, 0, 0.1)


if __name__ == '__main__':
	main(sys.argv)
