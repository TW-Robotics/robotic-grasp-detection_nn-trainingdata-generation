#!/usr/bin/env python
import numpy as np
import sys
import random
import rospy
import tf
import math
import os
import csv
import json
import re

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ur5_control import ur5_control

debug = False
if rospy.get_param("print_debug") == True:
	print "Debug-Mode ON"
	debug = True		# Print Debug-Messages
storePC = False		# Store Point-Cloud-Message

class dataCapture():
	def __init__(self, path):
		# Init variables
		self.goals = PoseArray()
		self.pc = PointCloud2()
		self.actPoseID = 0
		self.lastPoseID = 0
		self.actStorage = -1
		self.rgb_img = None
		self.d_img = None
		self.camera_settings_rgb = None
		self.camera_settings_depth = None
		self.intrinsics = [0., 0., 0., 0.]
		self.objectName = "carrier"

		# Instantiate CvBridge
		self.bridge = CvBridge()

		##################################
		# Give parameters in deg, meters #
		##################################
		# Path to store images and stuff
		#self.path = "/home/johannes/catkin_ws/src/data/"
		self.path = path

		# Parameters for randomization
		self.rotateTiltRMin = rospy.get_param("PoseRandomization/rotate4Min")#-10 	# Joint 4: How far to rotate
		self.rotateTiltRMax = rospy.get_param("PoseRandomization/rotate4Max")#10
		self.rotateUpRMin = rospy.get_param("PoseRandomization/rotate5Min")#-10 	# Joint 5: How far to rotate
		self.rotateUpRMax = rospy.get_param("PoseRandomization/rotate5Max")#10
		self.rotateRMin = rospy.get_param("PoseRandomization/rotate6Min")#-65		# Joint 6: How far can EEF be rotated
		self.rotateRMax = rospy.get_param("PoseRandomization/rotate6Max")#65
		##################################
		# ## # # # # # # # # # # # # # # #
		##################################

		# Camera Info
		self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoRGB_callback, queue_size=1) 
		self.d_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cameraInfoD_callback, queue_size=1) 

		# Images
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)					# RGB-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.d_image_callback)	# Depth-Image

		# Poses
		rospy.Subscriber("/capturePoses", PoseArray, self.pose_callback, queue_size=1)		# Poses to drive to

		#self.ur5 = ur5_control.ur5Controler("camera_planning_frame", "/object_img_center", False)

		self.listener = tf.TransformListener()
		rospy.sleep(1)		# Wait for topics to arrive
		
		# Write camera and object-scene settings if camera-infos arrived and base-to-object-transformation already arrived
		self.write_cam_settings()
		self.write_scene_settings()

		'''rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			print "nothing"
			rate.sleep()'''

	###########################################################
	# Functions to get dicts ##################################
	###########################################################

	# Store camera-parameters in dict
	def get_camera_dict(self, w, h, fx, fy, cx, cy, name, fov):
		captured_image_size = {"width": w, "height": h}
		intrinsic_settings = {"resX": w, "resY": h, "fx": fx, "fy": fy, "cx": cx, "cy": cy, "s": 0}
		self.camera_settings_rgb = {"name": name, "horizontal_fov": fov, "intrinsic_settings": intrinsic_settings, "captured_image_size": captured_image_size}
		return cam_settings

	def get_data_dict(self):
		''' # DEBUG
		baseToCam = Pose()
		baseToCam.position.x = 0
		baseToCam.position.y = 1
		baseToCam.position.z = 0.5
		baseToCam.orientation.x = 0.707
		baseToCam.orientation.y = -0.707
		baseToCam.orientation.z = 0
		baseToCam.orientation.w = 1
		camToObj = baseToCam
		objectName = "carrier"'''

		''' # DEBUG LOAD
		with open(str(self.path) + '000000.json') as json_file:  
			data = json.load(json_file)
		cd = data["camera_data"]
		print cd["location_worldframe"]'''

		while True:
			baseToCam = self.get_transform_baseCam()
			camToObj = self.get_transform_camObj()
			if baseToCam is not None and camToObj is not None:
				break
			print "Waiting for transformations to arrive..."
			rospy.sleep(0.05)

		# Camera Data
		location_worldframe = [baseToCam.position.x*100, baseToCam.position.y*100, baseToCam.position.z*100]
		quaternion_xyzw_worldframe = [baseToCam.orientation.x, baseToCam.orientation.y, baseToCam.orientation.z, baseToCam.orientation.w]
		camera_data = {'location_worldframe': location_worldframe, 'quaternion_xyzw_worldframe': quaternion_xyzw_worldframe}

		# Get Cuboids
		cuboidPoses = self.get_cuboid_transforms()
		cuboidProj = self.calculate_projection(cuboidPoses, self.intrinsics)

		# Object Data
		location = [camToObj.position.x*100, camToObj.position.y*100, camToObj.position.z*100]
		quaternion_xyzw = [camToObj.orientation.x, camToObj.orientation.y, camToObj.orientation.z, camToObj.orientation.w]
		pose_transform = self.get_permuted_matrix_from_pose(camToObj)
		
		cuboid_centroid = [cuboidPoses.poses[8].position.x, cuboidPoses.poses[8].position.y, cuboidPoses.poses[8].position.z]
		projected_cuboid_centroid = [cuboidProj.poses[8].position.x, cuboidProj.poses[8].position.y]
		bounding_box = {"top_left": "NaN", "bottom_right": "NaN"}
		cuboid = []
		for i in range(len(cuboidPoses.poses) - 1):
			cuboid.append([cuboidPoses.poses[i].position.x*100, cuboidPoses.poses[i].position.y*100, cuboidPoses.poses[i].position.z*100])
		projected_cuboid = []
		for i in range(len(cuboidProj.poses) - 1):
			projected_cuboid.append([cuboidProj.poses[i].position.x*100, cuboidProj.poses[i].position.y*100])
		objects = {"class": self.objectName, "instance_id": 0, "visibility": 1, "location": location, "quaternion_xyzw": quaternion_xyzw,
					"pose_transform": pose_transform, "cuboid_centroid": cuboid_centroid, "projected_cuboid_centroid": projected_cuboid_centroid,
					"cuboid": cuboid, "projected_cuboid": projected_cuboid}

		data = {'camera_data': camera_data, 'objects': objects}
		return data

	# Get permuted transformation matrix for fat-dataset-format
	def get_permuted_matrix_from_pose(self, pose):
		# INFO:
		# Segmentation-id = 255 since only 'Single'-images are produced (see FAT readme)
		# Cuboid-Dimensions in cm extracted from dataset-synthesizer json-file
		quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
		M_rot = tf.transformations.quaternion_matrix(quat)
		M_res = np.delete(M_rot, 3, 1)	# Delete last column
		M_res = np.delete(M_res, 3, 0)	# Delete last row
		P_inv = np.linalg.inv([[0, 0, 1],[1, 0, 0],[0, -1, 0]])
		M_out = np.matmul(M_res, P_inv).transpose()

		transl = [pose.position.x*100, pose.position.y*100, pose.position.z*100, 1]
		M_out = np.c_[M_out, np.zeros(3)]		# Add empty column as last column
		M_out = np.append(M_out, values=transl)	# Add translation as last row
		return M_out

	# Collect all info for camera-setting-file and write it
	def write_cam_settings(self):
		while True:
			if (self.camera_settings_rgb is not None and self.camera_settings_depth is not None):
				break
			print ("Waiting for camera-settings to arrive...")
			rosply.sleep(0.5)
		data = {"camera_settings": [self.camera_settings_rgb, self.camera_settings_depth]}
		self.write_json(data, "_camera_settings.json")

	# Collect all info for object-setting-file and write it
	def write_scene_settings(self):
		while True:
			baseObjPose = self.get_transform_baseObj()
			if baseObjPose is not None:
				break
			print "Waiting for Base-Object-Transform to arrive..."
			rospy.sleep(0.5)

		fixed_model_transform = self.get_permuted_matrix_from_pose(baseObjPose)
		exported_objects = {"class": self.objectName, "segmentation_class_id": 255, "segmentation_instance_id": 255, "fixed_model_transform": fixed_model_transform, "cuboid_dimensions": [19.8, 90.0, 177.0]}
		data = {"exported_object_classes": [self.objectName], "exported_objects": [exported_objects]}
		self.write_json(data, "_object_settings.json")

	# Write data to json-file (formatted)
	def write_json(self, data, filename):
		dump = json.dumps(data, sort_keys=False, indent=4)
		new_data = re.sub('\n +', lambda match: '\n' + '\t' * (len(match.group().strip('\n')) / 3), dump)
		print >> open(str(self.path) + str(filename), 'w'), new_data

	###########################################################
	# CALLBACKS ###############################################
	###########################################################

	def cameraInfoRGB_callback(self, data):
		fx = data.K[0]#925.112183
		fy = data.K[4]#925.379517
		cx = data.K[2]#647.22644
		cy = data.K[5]#357.068359
		self.intrinsics = [fx, fy, cx, cy]
		#print self.fx, self.fy, self.cx, self.cy

		# Store camera-parameters in dict
		self.camera_settings_rgb = self.get_camera_dict(data.width, data.height, fx, fy, cx, cy, "RealsenseD435_RGB", 69.400001525878906)

		# Unregister from Camera-Info
		self.rgb_info_sub.unregister()

	def cameraInfoD_callback(self, data):
		fx = data.K[0]
		fy = data.K[4]
		cx = data.K[2]
		cy = data.K[5]

		# Store camera-parameters in dict
		self.camera_settings_depth = self.get_camera_dict(data.width, data.height, fx, fy, cx, cy, "RealsenseD435_Depth", 69.400001525878906)	#TODO check FOV

		# Unregister from Camera-Info
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

	# Capture-Poses
	def pose_callback(self, data):
		self.goals = data

	###########################################################
	# TRANSFORMATIONS #########################################
	###########################################################

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

	def poseToList(self, pose):
		l = []
		l.append(pose.position.x)
		l.append(pose.position.y)
		l.append(pose.position.z)
		l.append(pose.orientation.x)
		l.append(pose.orientation.y)
		l.append(pose.orientation.z)
		l.append(pose.orientation.w)
		return l

	def get_transform_baseObj(self):
		try:
			(trans, rot) = self.listener.lookupTransform('/base_link', '/object', rospy.Time(0))					# from base to object
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)
			return None

	def get_transform_baseCam(self):
		try:
			(trans, rot) = self.listener.lookupTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0))# transform from base to camera
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)		
			return None

	def get_transform_camObj(self):
		try:
			(trans, rot) = self.listener.lookupTransform('/camera_color_optical_frame', '/object', rospy.Time(0))	# transform from camera to object
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)
			return None

	def get_transformations(self):
		baseObjPose = self.get_transform_baseObj()
		baseCamPose = self.get_transform_baseCam()
		camObjPose = self.get_transform_camObj()
		try:
			(trans, rot) = self.listener.lookupTransform('/object', '/camera_color_optical_frame', rospy.Time(0))	# from object to camera
			objCamPose = self.listToPose(trans, rot)
			(trans, rot) = self.listener.lookupTransform('/base_link', '/tool0_controller', rospy.Time(0))			# transform from base to tool0_controller
			baseToolPose = self.listToPose(trans, rot)
			return baseObjPose, baseCamPose, camObjPose, objCamPose, baseToolPose
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)
			return None

	def get_cuboid_transforms(self):
		cuboidPoses = PoseArray()
		for i in range(9):
			try:
				(trans, rot) = self.listener.lookupTransform('/camera_color_optical_frame', '/c_'+str(i), rospy.Time(0))
				cuboidPoses.poses.append(self.listToPose(trans, rot))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr(e)
		return cuboidPoses

	def calculate_projection(self, cuboidPoses, intrinsics):
		cuboidProj = PoseArray()
		for i in range(len(cuboidPoses.poses)):
			p = Pose()
			p.position.x = intrinsics[2] + 1/cuboidPoses.poses[i].position.z * (intrinsics[0] * cuboidPoses.poses[i].position.x)
			p.position.y = intrinsics[3]+ 1/cuboidPoses.poses[i].position.z * (intrinsics[1] * cuboidPoses.poses[i].position.y)
			cuboidProj.poses.append(p)
		return cuboidProj

	###########################################################
	# ROBOT-RELATED ###########################################
	###########################################################

	def drive_to_pose(self, id):
		self.ur5.execute_move(self.goals.poses[id])

	# Make random moves with last axis
	def move_random(self):
		# Sample random offsets
		rotateUp = random.uniform(self.rotateUpRMin, self.rotateUpRMax)
		rotateDown = random.uniform(-self.rotateUpRMin, -self.rotateUpRMax)
		rotateTiltL = random.uniform(self.rotateTiltRMin, self.rotateTiltRMax)
		rotateTiltR = random.uniform(-self.rotateTiltRMin, -self.rotateTiltRMax)

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

	###########################################################
	# STORE DATA ##############################################
	###########################################################

	# Store images and poses at actual position to json-files
	def store_state(self):
		# Calculate actual name-prefix for image
		if self.lastPoseID == self.actPoseID:
			self.actStorage = self.actStorage + 1
		else:
			self.lastPoseID = self.actPoseID
			self.actStorage = 0
		namePreFix = str(self.actPoseID) + "_" + str(self.actStorage)

		# Make sure to refresh data -> Set None and wait until new images arrive
		self.d_img = None
		self.rgb_img = None
		while self.d_img is None or self.rgb_img is None:
			print "Waiting for images to arrive..."
			rospy.sleep(0.05)

		#output = np.hstack((cv2.cvtColor(self.d_img, cv2.COLOR_GRAY2BGR), self.rgb_img))
		output = self.rgb_img
		cv2.imshow("Images", output)
		cv2.waitKey(1)

		# Store Images
		# Source: https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
		d_img = self.d_img.copy()
		rgb_img = self.rgb_img.copy()

		# Save OpenCV2 images
		cv2.imwrite(str(self.path) + str(namePreFix) + "_rgb.png", rgb_img)
		cv2.imwrite(str(self.path) + str(namePreFix) + "_d.png", d_img*255)	# *255 to rescale from 0-1 to 0-255

		#cv2.imshow("Images", cv2_img)
		#cv2.waitKey(1)

		# Store Depth-Image as CSV-File
		with open(str(self.path) + str(namePreFix) + "_d.csv", "wb") as f:
			writer = csv.writer(f, delimiter=";")
			for line in d_img:
				writer.writerow(line)

		# Store Depth-Image as Point-Cloud
		if storePC == True:
			f1 = open(str(self.path) + str(namePreFix) + "pc.ply", "w")
			f1.write("ply\nformat ascii 1.0\nelement vertex 921600\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
			for row in range(len(d_img)):			#1280
				for col in range(len(d_img[0])):	#720
					f1.write(str(float(row) / 1000.) + " " + str(float(col) / 1000.) + " " + str(float(d_img[row][col]) / 1000.) + "\n")
			f1.close()

		print_debug("RGB and Depth-Data Stored " + str(namePreFix))

		data = self.get_data_dict(self.objectName)
		self.write_json(data, str(namePreFix) + "_000000.json")	# TODO NAME!

		''' # DEBUG
		baseObjPose, baseCamPose, camObjPose, objCamPose, baseToolPose = self.get_transformations()
		data = {"camToObj_xyz_xyzw": self.poseToList(camObjPose),
				"objToCam_xyz_xyzw": self.poseToList(objCamPose),
				"baseToObj_xyz_xyzw": self.poseToList(baseObjPose),
				"baseTotool0_controller_xyz_xyzw": self.poseToList(baseToolPose),
				"baseToCam_xyz_xyzw": self.poseToList(baseCamPose),}
		self.write_json(data, str(namePreFix) + "_poses.json")'''

		print "Stored " + str(namePreFix)

	# Drive to the goals and make random moves
	def capture(self, startID):
		'''i = 0
		while True:
			self.actPoseID = i
			inp = raw_input("y to Store, e to Exit, n to continue: ")[0]
			if inp == 'y':
				self.store_state()
				print "done"
			elif inp == 'e':
				return
			i = i + 1'''

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

			rotateRand = random.uniform(self.rotateRMin, self.rotateRMax)
			print_debug("Rotating1 " + str(rotateRand))
			self.ur5.move_joint(5, rotateRand)				# Rotate the EEF
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(-self.rotateRMin, -self.rotateRMax)
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
		path = rospy.get_param("path_to_store")#"/home/mluser/catkin_ws/src/data/"
		path = path + str(args[1]) + "/"
		if not os.path.exists(path):
				os.makedirs(path)
		else:
			rospy.logwarn("You are writing to an existing folder!")
		dc = dataCapture(path)
	startID = 0
	if len(args) == 3:
		startID = int(args[2])
		print "Starting at pose no. " + str(startID)
	#dc.drive_to_pose(1)

	'''while True:
		dc.actPoseID = 0
		inp = raw_input("y to Store, e to Exit, n to continue: ")[0]
		if inp == 'y':
			dc.actPoseID = dc.actPoseID + 1
			dc.store_state()
		elif inp == 'e':
			return'''

	rospy.logwarn("Don't forget to put away the marker!")

	dc.capture(startID)

if __name__ == '__main__':
	main(sys.argv)
