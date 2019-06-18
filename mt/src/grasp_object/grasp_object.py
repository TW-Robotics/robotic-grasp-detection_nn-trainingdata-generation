#!/usr/bin/env python
import numpy as np
import sys
import select
import rospy
import tf
import math
import collections
import csv
import time
from math import pi
import argparse

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
	def __init__(self, side):
		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()
		self.tfBroadcaster = tf.TransformBroadcaster()

		self.ur5 = ur5_control.ur5Controler("gripper", "/base_link", True)
		self.gripper = gripper_control.gripper()
		self.side = side

		self.graspPose = Pose()
		self.preGraspPose = Pose()
		self.rgb_img = Image()
		self.poseIsUpdated = False

		self.publishResized = True

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
			self.img_raw = data
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

	def get_pose(self, fromF, toF):
		try:
			(trans, rot) = self.tfListener.lookupTransform(fromF, toF, rospy.Time(0))
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

	# Get transformation between base and grasp-point
	def update_grasp_pose(self):
		rospy.sleep(0.1)
		self.graspPose = self.get_pose('/base_link', 'dope_grasp_point')
		self.preGraspPose = self.get_pose('/base_link', 'dope_grasp_point_pre')
		self.poseIsUpdated = True

	def publish_image(self):
		self.poseIsUpdated = False
		if self.publishResized == False:
			self.rawImgPub.publish(self.img_raw)
		else:
			self.rawImgPub.publish(self.bridge.cv2_to_imgmsg(self.rgb_img_resized, "bgr8"))

	def prepare_grasp(self):
		actJointValues = self.ur5.group.get_current_joint_values()
		# Pre-position last joints so robot does not jeopardize environment 
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], -240*pi/180, -85*pi/180, 0*pi/180])
		self.ur5.execute_move([actJointValues[0], -90*pi/180, 150*pi/180, actJointValues[3], actJointValues[4], actJointValues[5]])

	def make_grasp(self):
		self.prepare_grasp()
		self.ur5.move_to_pose(self.preGraspPose)
		self.ur5.move_to_pose(self.graspPose)
		rospy.sleep(5)

		##### Close the gripper to grasp object
		self.gripper.close()
		rospy.sleep(5)
		if self.gripper.hasGripped() == True:
			print "Successfully grasped object!"
		else:
			print "Error grasping object!"
			return False

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
	parser = argparse.ArgumentParser(description='Grasp Object')
	parser.add_argument('Side', metavar='Side', type=str, help='Side to search for object [front, left, right]')
	side = parser.parse_args().Side

	# Init node
	rospy.init_node('grasp_object', anonymous=True, disable_signals=True)

	grasper = grasp_process(side)

	rospy.loginfo("Make sure correct camera intrinsics are set!")

	while (True):
		inp = raw_input("p to publish image, g to drive to grasp pose, h for p+g, a for automatic: ")[0]
		if inp == 'p':
			grasper.publish_image()
		elif inp == 'g':
			if grasper.poseIsUpdated == True:
				if grasper.make_grasp() == True:
					##### Move the robot up and to transport-pose
					grasper.ur5.move_xyz(0, 0, 0.2)
					grasper.ur5.moveToTransportPose()
			else:
				print "Pose not updated. Press 'p' first"

		elif inp == 'h':
			grasper.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while grasper.poseIsUpdated == False:
				rospy.sleep(0.05)
				if time.time() >= timeout
					print "Error locating object"
					break
			else:
				print "Received pose update - driving to object"
				if grasper.make_grasp() == True:
					##### Move the robot up and to transport-pose
					grasper.ur5.move_xyz(0, 0, 0.2)
					grasper.ur5.moveToTransportPose()	# TODO Change joint angles

		elif inp == 'a':
			grasper.ur5.moveToSearchPose(grasper.side) 	# TODO Change search-position so robot sees more
			rospy.sleep(0.1)	# wait to arrive at position
			grasper.publish_image()
			posID = 0
			rospy.sleep(1) 		# wait for image to arrive
			while grasper.poseIsUpdated == False:		# if the object has been found, this is going to be True and outer loop is exited
				if grasper.ur5.searchObject(posID) == False:
					print "No object found!"
					break
				posID = posID + 1
				rospy.sleep(0.1) 	# wait to arrive at position
				grasper.publish_image()
				timeout = time.time() + 1.5   # 1.5 seconds from now
				# Wait until updated pose arrives or timeout occurs (pose not visible)
				while grasper.poseIsUpdated == False:
					rospy.sleep(0.05)
					if time.time() >= timeout:
						print "Object not found - moving on..."
						break
				else:
					print "Received pose update - driving to object."
					if grasper.make_grasp() == True:
						##### Move the robot up and to transport-pose
						grasper.ur5.move_xyz(0, 0, 0.2)
						grasper.ur5.moveToTransportPose()

if __name__ == '__main__':
	main(sys.argv)


	'''pose = Pose()
	pose.position.x = 0.6
	pose.position.y = 0.1
	pose.position.z = 0.37
	pose.orientation.x = 0.#-0.4778
	pose.orientation.y = 0.707#0.4731
	pose.orientation.z = 0#-0.5263
	pose.orientation.w = 0.707#0.5203
	goalPose = [0.6, 0.1, 0.37, 0, 0, 0.707, 0.707]'''