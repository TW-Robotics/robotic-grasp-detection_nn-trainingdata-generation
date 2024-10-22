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
from mission_control import mission_control

debug = False
if rospy.get_param("print_debug") == True:
	print "Debug-Mode ON"
	debug = True		# Print Debug-Messages

class transport_process():
	def __init__(self):
		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init Listener for tf-transformation
		self.tfListener = tf.TransformListener()
		self.tfBroadcaster = tf.TransformBroadcaster()

		self.ur5 = ur5_control.ur5Controler("gripper", "/base_link", False)
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
		self.graspPose = self.get_pose('/base_link', 'dope_grasp_pose_carrier')
		self.preGraspPose = self.get_pose('/base_link', 'dope_grasp_pose_carrier_pre')
		self.poseIsUpdated_carrier = True

	def update_put_pose(self):
		rospy.sleep(0.1)
		self.putPose = self.get_pose('/base_link', 'dope_put_pose_holder')
		self.prePutPose = self.get_pose('/base_link', 'dope_put_pose_holder_pre')
		self.postPutPose = self.get_pose('/base_link', 'dope_put_pose_holder_post')
		self.poseIsUpdated_holder = True		

	def publish_image(self):
		self.poseIsUpdated_carrier = False
		self.poseIsUpdated_holder = False
		if self.publishResized == False:
			self.rawImgPub.publish(self.img_raw)
		else:
			self.rawImgPub.publish(self.bridge.cv2_to_imgmsg(self.rgb_img_resized, "bgr8"))

	def prepare_grasp(self):
		actJointValues = self.ur5.group.get_current_joint_values()
		# Pre-position last joints so robot does not jeopardize environment 
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], -240*pi/180, -85*pi/180, 0*pi/180])
		actJointValues = self.ur5.group.get_current_joint_values()
		self.ur5.execute_move([actJointValues[0], -90*pi/180, 150*pi/180, actJointValues[3], actJointValues[4], actJointValues[5]])

	def prepare_put(self, side):
		print "Moving joints in put-configuration..."
		if side == "front":
			r1_angle = 0
		elif side == "left":
			r1_angle = 90
		elif side == "right":
			r1_angle = -90
		actJointValues = self.ur5.group.get_current_joint_values()
		self.ur5.execute_move([r1_angle*pi/180, actJointValues[1], actJointValues[2], actJointValues[3], actJointValues[4], actJointValues[5]])

	def make_grasp(self):
		print "Moving joints in grasp-configuration..."
		self.prepare_grasp()
		print "Driving to object"
		self.ur5.move_to_pose(self.preGraspPose)
		self.ur5.move_to_pose(self.graspPose)

		##### Close the gripper to grasp object
		self.gripper.close()
		self.ur5.attachObjectToEEF()
		rospy.sleep(5)
		if self.gripper.hasGripped() == True:
			print "Successfully grasped object!"
		else:
			print "Error grasping object!"
			return False
		self.hasGraspedPub.publish(Bool(True))
		#rospy.sleep(0.5)
		return True

	def put_down(self, side):
		self.prepare_put(side)
		print "Driving to put-down-position"
		self.ur5.move_to_pose(self.prePutPose)
		self.ur5.move_to_pose(self.putPose)

		##### Open the gripper
		self.gripper.open()
		self.ur5.removeAttachedObject()
		rospy.sleep(5)
		self.hasPutPub.publish(Bool(True))
		#self.ur5.removeAttachedObject()
		return True

	def store(self):
		actJointValues = self.ur5.group.get_current_joint_values()
		# Einfahren
		self.ur5.execute_move([actJointValues[0], -71*pi/180, 153*pi/180, -263*pi/180, -90*pi/180, 0*pi/180])
		# Drehen
		actJointValues = self.ur5.group.get_current_joint_values()
		self.ur5.execute_move([90*pi/180, actJointValues[1], actJointValues[2], actJointValues[3], actJointValues[4], actJointValues[5]])
		# Ausfahren
		self.ur5.execute_move([90*pi/180, -53*pi/180, 117*pi/180, -244*pi/180, -90*pi/180, 0*pi/180])
		# Vorletzte Achse drehen
		actJointValues = self.ur5.group.get_current_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], actJointValues[3], 0*pi/180, actJointValues[5]])
		# Drive over position
		self.ur5.execute_move([121*pi/180, -49*pi/180, 106*pi/180, -240*pi/180, 57*pi/180, 0*pi/180])
		# Drive to put down position
		self.ur5.execute_move([121*pi/180, -41*pi/180, 106*pi/180, -248*pi/180, 57*pi/180, 0*pi/180])

		##### Open the gripper
		self.gripper.open()
		self.ur5.removeAttachedObject()
		rospy.sleep(5)
		self.hasPutPub.publish(Bool(True))
		#self.ur5.removeAttachedObject()

		# Vorletzte Achse drehen
		self.ur5.execute_move([121*pi/180, -41*pi/180, 106*pi/180, -248*pi/180, 35*pi/180, 0*pi/180])

	def unstore(self):
		# In Nebenposition bewegen
		self.ur5.execute_move([121*pi/180, -41*pi/180, 106*pi/180, -248*pi/180, 35*pi/180, 0*pi/180])
		# Hineinschwenken
		self.ur5.execute_move([121*pi/180, -41*pi/180, 106*pi/180, -248*pi/180, 57*pi/180, 0*pi/180])

		self.gripper.close()
		self.ur5.attachObjectToEEF()
		rospy.sleep(5)
		if self.gripper.hasGripped() == True:
			print "Successfully grasped object!"
		else:
			print "Error grasping object!"
			return False
		self.hasGraspedPub.publish(Bool(True))
		
		# lift
		self.ur5.move_xyz(0, 0, 0.05)
		# turn
		self.ur5.execute_move([90*pi/180, -53*pi/180, 117*pi/180, -244*pi/180, 0*pi/180, 0*pi/180])
		# Vorletzte Achse drehen
		actJointValues = self.ur5.group.get_current_joint_values()
		self.ur5.execute_move([actJointValues[0], actJointValues[1], actJointValues[2], actJointValues[3], -90*pi/180, actJointValues[5]])

		return True

	def move_and_publish(self, jointID, angle):
		self.ur5.move_joint(jointID, angle)
		rospy.sleep(0.1)
		self.publish_image()
		timeout = time.time() + 1.5
		# Wait until updated pose arrives or timeout occurs (pose not visible).
		# publish_image sets both poseIsUpdated to False. If an object is detected it is set True in the according callback and this function returns True
		while self.poseIsUpdated_carrier == False and self.poseIsUpdated_holder == False:
			rospy.sleep(0.05)
			if time.time() >= timeout:
				print "Object not detectable"
				return False
		return True

	def refine_pose(self):
		print "Refining pose..."
		self.move_and_publish(5, 20)
		self.move_and_publish(5, -40)
		self.ur5.move_joint(5, 20) 	# move back to base position
		self.move_and_publish(4, 10)
		self.move_and_publish(4, -20)

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
	parser.add_argument('pickUpGoal', metavar='pickUpGoal', type=str, help='Name of goal to pick up carrier')
	parser.add_argument('putDownGoal', metavar='putDownGoal', type=str, help='Name of goal to put down carrier')
	pickUpGoal = mission_control.goal(parser.parse_args().pickUpGoal)
	putDownGoal = mission_control.goal(parser.parse_args().putDownGoal)
	
	# Init node
	rospy.init_node('transport_object', anonymous=True, disable_signals=True)

	transporter = transport_process()

	rospy.loginfo("Make sure correct camera intrinsics are set in yaml file!")

	while (True):
		transporter.reset_poses()
		inp = raw_input("p to publish image, c for grasping, h for putting, a for search+grasp, b for search+put: ")[0]
		
		if inp == 's':
			transporter.store()

		if inp == 'u':
			transporter.unstore()

		if inp == 'p':
			transporter.publish_image()

		elif inp == 'c':
			transporter.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while transporter.poseIsUpdated_carrier == False:
				rospy.sleep(0.05)
				if time.time() >= timeout:
					print "Error locating object"
					break
			else:
				print "Received pose update"
				transporter.refine_pose()
				transporter.ur5.addMesh(transporter.showObjPose,"/dope_object_pose_carrier")
				if transporter.make_grasp() == True:
					##### Move the robot up and to transport-pose
					transporter.ur5.move_xyz(0, 0, 0.05)
					#transporter.ur5.moveToTransportPose()

		elif inp == 'h':
			transporter.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while transporter.poseIsUpdated_holder == False:
				rospy.sleep(0.05)
				if time.time() >= timeout:
					print "Error locating object"
					break
			else:
				print "Received pose update"
				transporter.refine_pose()
				if transporter.put_down(putDownGoal.orientation) == True:
					##### Move the robot up and to transport-pose
					transporter.ur5.move_to_pose(transporter.postPutPose)
					#transporter.ur5.moveToTransportPose()			

		elif inp == 'a':
			transporter.hasGraspedPub.publish(Bool(True))
			transporter.hasPutPub.publish(Bool(True))
			transporter.ur5.removeAttachedObject()
			transporter.ur5.scene.remove_world_object()
			transporter.ur5.moveToSearchPose(pickUpGoal.orientation)
			rospy.sleep(0.1)	# wait to arrive at position
			transporter.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while transporter.poseIsUpdated_carrier == False and time.time() <= timeout:
				rospy.sleep(0.05)
			posID = 0
			while transporter.poseIsUpdated_carrier == False:		# if the object has been found, this is going to be True and outer loop is exited
				if transporter.ur5.searchObject(posID) == False:
					print "No object found!"
					break
				posID = posID + 1
				rospy.sleep(0.1) 	# wait to arrive at position
				transporter.publish_image()
				timeout = time.time() + 1.5   # 1.5 seconds from now
				# Wait until updated pose arrives or timeout occurs (pose not visible)
				while transporter.poseIsUpdated_carrier == False:
					rospy.sleep(0.05)
					if time.time() >= timeout:
						print "Object not found - moving on..."
						break
			else:
				print "Received pose update"
				transporter.refine_pose()
				transporter.ur5.addMesh(transporter.showObjPose,"/dope_object_pose_carrier")
				if transporter.make_grasp() == True:
					##### Move the robot up and to transport-pose
					transporter.ur5.move_xyz(0, 0, 0.05)
					#transporter.ur5.moveToTransportPose()

		elif inp == 'b':
			transporter.hasGraspedPub.publish(Bool(True))
			transporter.hasPutPub.publish(Bool(True))
			transporter.ur5.moveToSearchPose(putDownGoal.orientation)
			rospy.sleep(0.1)	# wait to arrive at position
			transporter.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while transporter.poseIsUpdated_holder == False and time.time() <= timeout:
				rospy.sleep(0.05)
			posID = 0
			while transporter.poseIsUpdated_holder == False:		# if the object has been found, this is going to be True and outer loop is exited
				if transporter.ur5.searchObject(posID) == False:
					print "No object found!"
					break
				posID = posID + 1
				rospy.sleep(0.1) 	# wait to arrive at position
				transporter.publish_image()
				timeout = time.time() + 1.5   # 1.5 seconds from now
				# Wait until updated pose arrives or timeout occurs (pose not visible)
				while transporter.poseIsUpdated_holder == False:
					rospy.sleep(0.05)
					if time.time() >= timeout:
						print "Object not found - moving on..."
						break
				else:
					print "Received pose update"
					transporter.refine_pose()
					if transporter.unstore() == True:
						if transporter.put_down(putDownGoal.orientation) == True:
							##### Move the robot up and to transport-pose
							transporter.ur5.move_to_pose(transporter.postPutPose)
							#transporter.ur5.moveToTransportPose()

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
