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

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

class grasp_process():
	def __init__(self):
		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()
		self.tfBroadcaster = tf.TransformBroadcaster()

		self.ur5 = ur5_control.ur5Controler("gripper", "/base_link_ur", False)
		self.gripper = gripper_control.gripper()

		self.reset_poses()
		self.rgb_img = Image()
		self.poseIsUpdated_carrier = False
		self.poseIsUpdated_holder = False
		
		self.showObjPose = Pose()
		self.showObjPose.position.z = -0.03
		self.showObjPose.position.y = -0.045
		self.showObjPose.orientation.y = -0.707
		self.showObjPose.orientation.w = 0.707

		self.publishResized = True

		self.imgOutputSize = rospy.get_param("outputImage_size")
		self.imgWidth = rospy.get_param("camera_width")
		self.imgHeight = rospy.get_param("camera_height")
		self.img_scaleFac = float(self.imgHeight)/self.imgOutputSize
		self.resized_imgWidth = int(round(float(self.imgWidth)/self.img_scaleFac, 0))
		self.resized_img_horizontalStart = int(round(self.resized_imgWidth/2., 0)) - self.imgOutputSize/2

		# Subscriber to pose update
		rospy.Subscriber("/dope/pose_update_carrier", Bool, self.pose_update_carrier_callback)
		rospy.Subscriber("/dope/pose_update_holder", Bool, self.pose_update_holder_callback)
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)					# RGB-Image
		self.rawImgPub = rospy.Publisher("/dope/camera_images", Image, queue_size=10)
		self.hasGraspedPub = rospy.Publisher("/dope/has_grasped", Bool, queue_size=10)		# Publish Signal so pose is no more published to tf by other program
		self.hasPutPub = rospy.Publisher("/dope/has_put", Bool, queue_size=10)				# Publish Signal so pose is no more published to tf by other program

		rospy.sleep(1)	# Wait to register at tf

	def reset_poses(self):
		self.graspPose = Pose()
		self.preGraspPose = Pose()
		self.putPose = Pose()
		self.prePutPose = Pose()
		self.postPutPose = Pose()

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
	def pose_update_carrier_callback(self, data):
		self.update_grasp_pose()

	# Get the last put-pose, if a new pose has been sent to tf
	def pose_update_holder_callback(self, data):
		self.update_put_pose()

	def get_pose(self, fromF, toF):
		try:
			(trans, rot) = self.tfListener.lookupTransform(fromF, toF, rospy.Time(0))
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

	# Get transformation between base and grasp-point
	def update_grasp_pose(self):
		rospy.sleep(0.1)
		self.graspPose = self.get_pose('/base_link_ur', 'dope_grasp_pose_carrier')
		self.preGraspPose = self.get_pose('/base_link_ur', 'dope_grasp_pose_carrier_pre')
		self.poseIsUpdated_carrier = True

	def update_put_pose(self):
		rospy.sleep(0.1)
		self.putPose = self.get_pose('/base_link_ur', 'dope_put_pose_holder')
		self.prePutPose = self.get_pose('/base_link_ur', 'dope_put_pose_holder_pre')
		self.postPutPose = self.get_pose('/base_link_ur', 'dope_put_pose_holder_post')
		self.poseIsUpdated_holder = True		

	def publish_image(self):
		self.poseIsUpdated_carrier = False
		self.poseIsUpdated_holder = False
		if self.publishResized == False:
			self.rawImgPub.publish(self.img_raw)
		else:
			self.rawImgPub.publish(self.bridge.cv2_to_imgmsg(self.rgb_img_resized, "bgr8"))

######################################################################
######################################################################
######################################################################

	def search(self, goal, obj):
		# Move ur to search position
		self.ur5.execute_move(goal.urJoints)
		# Check if object is detected
		if self.check_for_object(obj) == False:
			# If object not detected search environment
			if self.search_around(obj) == False:
				print "No object found!"
				return False
		print "Object found!"
		return True

	def search_around(self, obj):
		vertInc = 10
		horiInc = 20
		moves =    [[0, horiInc], 	# right
					[3, vertInc],	# up
					[0, -horiInc], 	# left
					[0, -horiInc],	# left
					[3, -vertInc],	# down
					[3, -vertInc],	# down
					[0, horiInc], 	# right
					[0, horiInc]]	# right

		return self.move_and_check(moves, obj)

	def move_and_check(self, moves, obj):
		for i in range(len(moves)):
			self.ur5.move_joint(moves[i][0], moves[i][1])
			if self.check_for_object(obj) == True:
				return True
		return False

	def check_for_object(self, obj):
		rospy.sleep(0.1)		# wait to arrive at position
		self.publish_image()
		timeout = time.time() + 1.5   # 1.5 seconds from now

		# Wait until updated pose arrives or timeout occurs (pose not visible)
		while time.time() <= timeout:
			if obj == "carrier":
				if self.poseIsUpdated_carrier == True:
					return True 
			elif obj == "holder":
				if self.poseIsUpdated_holder == True:
					return True
			rospy.sleep(0.05)

		# If object not detected in image
		return False

	def refine_pose(self, obj):
		vert = 20
		hori = 7
		moves =    [[5, vert], 		# up
					[5, -2*vert],	# down
					[5, vert], 		# up to base
					[4, hori],		# right
					[3, -2*hori]]	# left

		self.move_and_check(moves, obj)

######################################################################
######################################################################
######################################################################

	def grasp(self):
		##### Close the gripper to grasp object
		print "Closing gripper..."
		self.gripper.close()
		self.ur5.attachObjectToEEF()
		rospy.sleep(5)
		if self.gripper.hasGripped() == True:
			self.hasGraspedPub.publish(Bool(True))
			print "Successfully grasped object!"
		else:
			print "Error grasping object!"
			return False
		return True

	def ungrasp(self):
		##### Open the gripper
		print "Opening gripper..."
		self.gripper.open()
		self.ur5.removeAttachedObject()
		rospy.sleep(5)
		self.hasPutPub.publish(Bool(True))

######################################################################
######################################################################
######################################################################

	def prepare_grasp(self):
		# Pre-position last joints so robot does not jeopardize environment
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], -240, -85, 0])
		# Move into pre-grasp position
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], -90, 150, actJointValues[3], actJointValues[4], actJointValues[5]])

	def make_grasp(self):
		print "Moving joints in grasp-configuration..."
		self.prepare_grasp()
		print "Driving to object..."
		self.ur5.move_to_pose(self.preGraspPose)
		self.ur5.move_to_pose(self.graspPose)

		if self.grasp == True:
			grasper.ur5.move_xyz_base_link_ur(0, 0, 0.05)
			return True

		# Open gripper and move to save position
		self.ungrasp()
		self.ur5.move_to_pose(self.preGraspPose)
		return False

######################################################################
######################################################################
######################################################################

	def prepare_put(self, side):
		if side == "front":
			r1_angle = 0
		elif side == "left":
			r1_angle = 90
		elif side == "right":
			r1_angle = -90
		# Move first joint to angle according to side to put object
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([r1_angle, actJointValues[1], actJointValues[2], actJointValues[3], actJointValues[4], actJointValues[5]])

	def put_down(self, side):
		print "Moving joints in put-configuration..."
		self.prepare_put(side)
		print "Driving to put-down-position..."
		self.ur5.move_to_pose(self.prePutPose)
		self.ur5.move_to_pose(self.putPose)

		self.ungrasp()
		grasper.ur5.move_to_pose(grasper.postPutPose)

######################################################################
######################################################################
######################################################################

	def store(self):
		# Move in
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], -71, 153, -263, -90, 0])
		# Turn to 90 deg position
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([90, actJointValues[1], actJointValues[2], actJointValues[3], actJointValues[4], actJointValues[5]])
		# Move out a little bit
		self.ur5.execute_move([90, -53, 117, -244, -90, 0])
		# Turn second last joint in
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], actJointValues[3], 0, actJointValues[5]])
		# Drive over put down position
		self.ur5.execute_move([121, -49, 106, -240, 57, 0])
		# Drive to put down position
		self.ur5.execute_move([121, -41, 106, -248, 57, 0])

		##### Open the gripper
		self.ungrasp()

		# Turn second last axis out for safe position
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], actJointValues[3], 35, actJointValues[5]])

	def unstore(self):
		# Move in position to schwenk in
		self.ur5.execute_move([121, -41, 106, -248, 35, 0])
		# Schwenk in
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], actJointValues[3], 57, actJointValues[5]])	# TODO Make position better

		if self.grasp() == False:
			# Move in safe position
			self.ur5.execute_move([121, -41, 106, -248, 35, 0])		
			return False

		# Lift object
		self.ur5.execute_move([121, -49, 106, -240, 57, 0])
		# Schwenk out
		self.ur5.execute_move([90, -53, 117, -244, 0, 0])
		# Turn second last joint out
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], actJointValues[3], -90, actJointValues[5]])
		# Move in a little bit
		actJointValues = self.ur5.get_joint_values()
		self.ur5.execute_move([actJointValues[0], -71, 153, -263, -90, 0])

		return True

######################################################################
######################################################################
######################################################################

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

def main(args):
	print "Use with transport_control.py!"
	return

if __name__ == '__main__':
	main(sys.argv)
