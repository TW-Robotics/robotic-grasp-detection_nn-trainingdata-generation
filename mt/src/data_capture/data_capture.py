#!/usr/bin/env python
import numpy as np
import sys
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d
import random

import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math
from geometry_msgs.msg import Quaternion
from ur5_control import ur5_control

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

#from capture_pose_calc import capture_pose_calc_structured as poseSampler

debug = True		# Print Debug-Messages

class dataCapture():
	def __init__(self):
		# Init node
		rospy.init_node('data_capture', anonymous=True, disable_signals=True)

		# Init variables
		self.goals = PoseArray()
		self.objBasePose = Pose()
		self.objCamPose = Pose()
		self.rgb_image = Image()
		self.d_image = Image()
		self.pc = PointCloud2()
		self.actPoseID = 0
		self.lastPoseID = 0
		self.actStorage = -1

		##################################
		# Give parameters in deg, meters #
		##################################
		# Path to store images and stuff
		#self.path = "/home/johannes/catkin_ws/src/mt/mt/src/data_capture/data/"
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

		rospy.Subscriber("/capturePoses", PoseArray, self.pose_callback, queue_size=1)		# Poses to drive to
		rospy.Subscriber("/tf_objToBase", Pose, self.objBasePose_callback, queue_size=1)		# Object-Pose w.r.t. robot
		rospy.Subscriber("/tf_objToCam", Pose, self.objCamPose_callback, queue_size=1)		# Object-Pose w.r.t. cam
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)			# RGB-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.d_image_callback)	# Depth-Image
		self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoRGB_callback, queue_size=1) 
		self.d_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cameraInfoD_callback, queue_size=1) 
		rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pc_callback)		# Point Cloud		

		self.ur5 = ur5_control.ur5Controler()

		# Instantiate CvBridge
		self.bridge = CvBridge()

		#rate = rospy.Rate(10)
		#while not rospy.is_shutdown():
		#	print "nothing"
		#	rate.sleep()

	def cameraInfoD_callback(self, data):
		f = open(str(self.path) + "depth-camera-info.txt", "w")
		f.write(str(data))
		f.close()
		self.d_info_sub.unregister()

	def cameraInfoRGB_callback(self, data):
		f = open(str(self.path) + "rgb-camera-info.txt", "w")
		f.write(str(data))
		f.close()
		self.rgb_info_sub.unregister()

	def pc_callback(self, data):
		self.pc = data
		#print "Got PC"
		#print type(self.pc)
		#print type(self.pc.data)
		#print self.pc.data

	# Subscribe to capture-poses
	def pose_callback(self, data):
		self.goals = data

	def rgb_image_callback(self, data):
		self.rgb_image = data
		#print self.rgb_image.encoding

	def d_image_callback(self, data):
		self.d_image = data

	# Subscribe to object pose
	def objBasePose_callback(self, data):
		self.objBasePose = data

	def objCamPose_callback(self, data):
		self.objCamPose = data

	# Make random moves with last axis
	def move_random(self):
		# Sample random offsets
		rotateUp = random.uniform(0, self.rotateUpRMax)
		rotateDown = random.uniform(self.rotateUpRMin, 0)
		rotateTiltL = random.uniform(0, self.rotateTiltRMax)
		rotateTiltR = random.uniform(self.rotateTiltRMin, 0)

		# Execute offsets
		print_debug("RotUp" + str(rotateUp))
		self.ur5.move_joint(4, rotateUp)
		print_debug("RotD" + str(rotateDown))
		self.ur5.move_joint(4, rotateDown - rotateUp)
		print_debug("TiltL" + str(rotateTiltL))
		self.ur5.move_joint(3, rotateTiltL)
		print_debug("TiltR" + str(rotateTiltR))
		self.ur5.move_joint(3, rotateTiltR - rotateTiltL)

	def store_state(self):
		# Source: https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
		# TODO Convert depth? http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html

		if self.lastPoseID == self.actPoseID:
			self.actStorage = self.actStorage + 1
		else:
			self.lastPoseID = self.actPoseID
			self.actStorage = 0
		namePreFix = str(self.actPoseID) + "_" + str(self.actStorage)

		try:
			# Convert your ROS Image message to OpenCV2
			rgb_img = self.bridge.imgmsg_to_cv2(self.rgb_image, "bgr8")
			d_img = self.bridge.imgmsg_to_cv2(self.d_image, "16UC1")
			# Save your OpenCV2 image
			cv2.imwrite(str(self.path) + str(namePreFix) + "_rgb.png", rgb_img)
			cv2.imwrite(str(self.path) + str(namePreFix) + "_d.png", d_img*255)	# *255 to rescale from 0-1 to 0-255
			#cv2.imshow("Grasp-Point", cv2_img)
			#cv2.waitKey(1)
			# Store Depth-Image as CSV-File
			f = open(str(self.path) + str(namePreFix) + "_d.csv", "w")
			for row in range(len(d_img)):			#1280
				for col in range(len(d_img[0])):	#720
					f.write(str(d_img[row][col]) + ";")
				f.write("\n")
			f.close()
			print_debug("Images Stored " + str(namePreFix))
			#cv2.ppf_match_3d.writePLY(self.pc.data, "test.ply")
		except CvBridgeError, e:
			print(e)

		# Store Object-to-Base-Pose and Object-to-Cam-Pose
		f = open(str(self.path) + str(namePreFix) + "_poses.txt", "w")
		f.write("Object to Cam:\n")
		f.write(str(self.objCamPose))
		f.write("\n\nObject to Base:\n")
		f.write(str(self.objBasePose))
		f.close()
		print_debug("Poses Stored")

	def drive_to_pose(self, id):
		self.ur5.execute_move(self.goals.poses[id])

	def capture(self):	# TODO add StartID
		# Drive to the goals and make random moves
		i = 0
		while True:
			self.actPoseID = i
			self.ur5.execute_move(self.goals.poses[i])		# Move to base-point
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point
			inp = raw_input("Store state? y/n: ")[0]
			if inp == 'y':
				self.store_state()
			elif inp == 'n':
				return

		for i in range(5):		
			'''self.ur5.execute_move(self.goals.poses[i])		# Move to base-point
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
			self.move_random()	'''							# Make random moves

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	dc = dataCapture()

	#dc.drive_to_pose(74)

	dc.capture()

if __name__ == '__main__':
	main(sys.argv)