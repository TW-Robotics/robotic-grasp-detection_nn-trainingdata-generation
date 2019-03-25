#!/usr/bin/env python
import numpy as np
import sys
import random
import rospy
import tf
import math
import os

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ur5_control import ur5_control

debug = True		# Print Debug-Messages
storePC = False		# Store Point-Cloud-Message

class dataCapture():
	def __init__(self):
		# Init variables
		self.goals = PoseArray()
		self.objBasePose = Pose()
		self.objCamPose = Pose()
		self.baseToolPose = Pose()
		self.pc = PointCloud2()
		self.actPoseID = 0
		self.lastPoseID = 0
		self.actStorage = -1
		self.rgb_img = None
		self.d_img = None

		# Instantiate CvBridge
		self.bridge = CvBridge()

		##################################
		# Give parameters in deg, meters #
		##################################
		# Path to store images and stuff
		#self.path = "/home/johannes/catkin_ws/src/data/"
		self.path = "/home/mluser/catkin_ws/src/data/"

		# Parameters for randomization
		self.rotateTiltRMin = -10 	# Joint 4: How far to rotate
		self.rotateTiltRMax = 10
		self.rotateUpRMin = -10 	# Joint 5: How far to rotate
		self.rotateUpRMax = 10
		self.rotateRMin = -65		# Joint 6: How far can EEF be rotated
		self.rotateRMax = 65
		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

		# Camera Info
		self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoRGB_callback, queue_size=1) 
		self.d_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cameraInfoD_callback, queue_size=1) 

		# Images
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)					# RGB-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.d_image_callback)	# Depth-Image
		#rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pc_callback)				# Point Cloud	

		# Poses
		rospy.Subscriber("/capturePoses", PoseArray, self.pose_callback, queue_size=1)		# Poses to drive to

		self.ur5 = ur5_control.ur5Controler("camera_planning_frame", "/object_img_center", True)

		self.listener = tf.TransformListener()
		rospy.sleep(1)		# Wait for topics to arrive

		'''rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			print "nothing"
			rate.sleep()'''

	# Images
	def cameraInfoRGB_callback(self, data):
		f = open(str(self.path) + "rgb-camera-info.txt", "w")
		f.write(str(data))
		f.close()
		self.rgb_info_sub.unregister()

	def cameraInfoD_callback(self, data):
		f = open(str(self.path) + "depth-camera-info.txt", "w")
		f.write(str(data))
		f.close()
		self.d_info_sub.unregister()

	def rgb_image_callback(self, data):
		try:
			cv_rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.rgb_img = cv_rgb_image.copy()
		except CvBridgeError as e:
			print(e)

	def d_image_callback(self, data):
		try:
			cv_depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			self.d_img = cv_depth_image.copy()
		except CvBridgeError as e:
			print(e)

		if self.rgb_img is not None:
			#output = np.hstack((cv2.cvtColor(self.d_img, cv2.COLOR_GRAY2BGR), self.rgb_img))
			output = self.rgb_img
			cv2.imshow("Images", output)
			cv2.waitKey(1)
		
	#def pc_callback(self, data):
		#self.pc = data

	# Capture-Poses
	def pose_callback(self, data):
		self.goals = data

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

	def get_transformations(self):
		try:
			# Get transformation and publish it rearranged to a list
			(trans, rot) = self.listener.lookupTransform('/base_link', '/object', rospy.Time(0))	# from base to object
			self.objBasePose = self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

		try:
			# Get transformation and publish it rearranged to a list
			(trans, rot) = self.listener.lookupTransform('/object', '/camera_color_optical_frame', rospy.Time(0))				# transform from object to camera
			self.objCamPose = self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

		try:
			# Get transformation and publish it rearranged to a list
			(trans, rot) = self.listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))				# transform from base to tool0_controller
			self.baseToolPose = self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

	# Make random moves with last axis
	def move_random(self):
		# Sample random offsets
		rotateUp = random.uniform(0, self.rotateUpRMax)
		rotateDown = random.uniform(self.rotateUpRMin, 0)
		rotateTiltL = random.uniform(0, self.rotateTiltRMax)
		rotateTiltR = random.uniform(self.rotateTiltRMin, 0)

		# Execute offsets
		self.store_state()
		print_debug("RotUp" + str(rotateUp))
		self.ur5.move_joint(4, rotateUp)
		self.store_state()
		print_debug("RotD" + str(rotateDown))
		self.ur5.move_joint(4, rotateDown - rotateUp)
		self.store_state()
		print_debug("TiltL" + str(rotateTiltL))
		self.ur5.move_joint(3, rotateTiltL)
		self.store_state()
		print_debug("TiltR" + str(rotateTiltR))
		self.ur5.move_joint(3, rotateTiltR - rotateTiltL)
		self.store_state()

	# Store images and poses
	def store_state(self):
		# Calculate actual name-prefix for image
		if self.lastPoseID == self.actPoseID:
			self.actStorage = self.actStorage + 1
		else:
			self.lastPoseID = self.actPoseID
			self.actStorage = 0
		namePreFix = str(self.actPoseID) + "_" + str(self.actStorage)
		rospy.sleep(0.5)
		# Store Images
		# Source: https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f

		# Save OpenCV2 images
		cv2.imwrite(str(self.path) + str(namePreFix) + "_rgb.png", self.rgb_img)
		cv2.imwrite(str(self.path) + str(namePreFix) + "_d.png", self.d_img*255)	# *255 to rescale from 0-1 to 0-255
		#cv2.imshow("Images", cv2_img)
		#cv2.waitKey(1)

		# Store Depth-Image as CSV-File
		f = open(str(self.path) + str(namePreFix) + "_d.csv", "w")
		for row in range(len(self.d_img)):			#1280
			for col in range(len(self.d_img[0])):	#720
				f.write(str(self.d_img[row][col]) + ";")
			f.write("\n")
		f.close()

		# Store Depth-Image as Point-Cloud
		if storePC == True:
			f1 = open(str(self.path) + str(namePreFix) + "pc.ply", "w")
			f1.write("ply\nformat ascii 1.0\nelement vertex 921600\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
			for row in range(len(self.d_img)):			#1280
				for col in range(len(self.d_img[0])):	#720
					f1.write(str(float(row) / 1000.) + " " + str(float(col) / 1000.) + " " + str(float(self.d_img[row][col]) / 1000.) + "\n")
			f1.close()

		print_debug("RGB and Depth-Data Stored " + str(namePreFix))

		self.get_transformations()
		# Store Object-to-Base-Pose and Object-to-Cam-Pose
		f = open(str(self.path) + str(namePreFix) + "_poses.txt", "w")
		f.write("object to camera_color_optical_frame:\n")
		f.write(str(self.objCamPose))
		f.write("\n\nbase_link to object:\n")
		f.write(str(self.objBasePose))
		f.write("\n\nbase_link to tool0_controller:\n")
		f.write(str(self.baseToolPose))
		f.close()
		print_debug("Poses Stored " + str(namePreFix))

	def drive_to_pose(self, id):
		self.ur5.execute_move(self.goals.poses[id])

	# Drive to the goals and make random moves
	def capture(self, startID):
		# TEST

		'''for i in range(len(self.goals.poses)):
			self.ur5.isReachable(self.goals.poses[i])
			inp = raw_input("Keep pose? ")[0]'''

		'''i = 0
		while True:
			self.actPoseID = i
			print_debug("drive to point " + str(i))
			self.ur5.execute_move(self.goals.poses[i])		# Move to base-point
			inp = raw_input("y to Store, e to Exit, n to continue: ")[0]
			if inp == 'y':
				self.store_state()
			elif inp == 'e':
				return
			i = i + 1'''

		for i in range(startID, len(self.goals.poses)):
			self.actPoseID = i		
			self.ur5.execute_move(self.goals.poses[i])		# Move to base-point
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(0, self.rotateRMax)
			print_debug("Rotating1 " + str(rotateRand))
			self.ur5.move_joint(5, rotateRand)				# Rotate the EEF
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(self.rotateRMin, 0)
			print_debug("Rotating2 " + str(rotateRand))
			self.ur5.move_joint(5, rotateRand)				# Rotate the EEF
			self.move_random()								# Make random moves

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init node
	rospy.init_node('data_capture', anonymous=True, disable_signals=True)

	if len(args) < 2:
		print "Please specify folder to store files!"
		return
	else:
		dc = dataCapture()
		dc.path = dc.path + str(args[1])
		if not os.path.exists(dc.path):
        		os.makedirs(dc.path)
		else:
			rospy.logwarn("You are writing to an existing folder!")
	startID = 0
	if len(args) == 3:
		startID = int(args[2])
		print "Starting at pose no. " + str(startID)
	#dc.drive_to_pose(1)

	rospy.logwarn("Don't forget to put away the marker!")

	dc.capture(startID)

if __name__ == '__main__':
	main(sys.argv)
