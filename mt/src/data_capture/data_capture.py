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
#from data_vis import *

debug = False
if rospy.get_param("print_debug") == True:
	print "Debug-Mode ON"
	debug = True		# Print Debug-Messages
storePC = False			# Store Point-Cloud-Message

class dataCapture():
	def __init__(self, path, pathFullRes):
		# Init variables
		self.goals = PoseArray()		# Poses to move robot to
		self.camPoses = []				# Store poses where camera was
		self.actStoreID = 0 			# Counter for file-name
		self.actPoseID = 0 				# Counter for poses
		self.lastPoseID = 0 			# Counter for poses to enable to increment actPoseID
		self.actStorage = -1 			# Counter for sub-poses
		self.path = path
		self.pathFullRes = pathFullRes

		# Images and Camera-Settings
		self.bridge = CvBridge()
		self.rgb_img = None				
		self.d_img = None
		self.rgb_img_resized = None
		self.camera_settings_rgb = None
		self.camera_settings_rgb_resized = None
		self.camera_settings_depth = None
		self.intrinsics = [0., 0., 0., 0.]
		self.intrinsics_resized = None

		# Parameters for randomization
		self.rotateTiltRMin = rospy.get_param("PoseRandomization/rotate4Min") 	# Joint 4: How far to rotate
		self.rotateTiltRMax = rospy.get_param("PoseRandomization/rotate4Max")
		self.rotateUpRMin = rospy.get_param("PoseRandomization/rotate5Min") 	# Joint 5: How far to rotate
		self.rotateUpRMax = rospy.get_param("PoseRandomization/rotate5Max")
		self.rotateRMin = rospy.get_param("PoseRandomization/rotate6Min")		# Joint 6: How far can EEF be rotated
		self.rotateRMax = rospy.get_param("PoseRandomization/rotate6Max")

		# Parameters for Image-Calculations
		self.objectName = rospy.get_param("object_name")
		self.imgOutputSize = rospy.get_param("outputImage_size")
		self.imgWidth = rospy.get_param("camera_width")
		self.imgHeight = rospy.get_param("camera_height")
		self.img_scaleFac = float(self.imgHeight)/self.imgOutputSize
		self.resized_imgWidth = int(round(float(self.imgWidth)/self.img_scaleFac, 0))
		self.resized_img_horizontalStart = int(round(self.resized_imgWidth/2., 0)) - self.imgOutputSize/2
		#print self.img_scaleFac, self.resized_imgWidth, self.resized_img_horizontalStart

		# Transformation-Listener
		self.listener = tf.TransformListener()

		# Subscribers
		self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoRGB_callback, queue_size=1) 
		self.d_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cameraInfoD_callback, queue_size=1) 
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)					# RGB-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.d_image_callback)	# Depth-Image
		rospy.Subscriber("/capturePoses", PoseArray, self.pose_callback, queue_size=1)				# Poses to drive to

		# Robot
		self.ur5 = ur5_control.ur5Controler("camera_planning_frame", "/object_img_center", False)

		# Wait for topics to arrive
		rospy.sleep(1)
		
		# Write camera and object-scene settings
		self.write_cam_settings()
		self.write_scene_settings()

	def draw_cuboids(self, img, cuboidProj):
		cuboidProj = np.asarray(cuboidProj)
		cuboidProj = cuboidProj.astype(int)
		for i in range(len(cuboidProj)):
			cv2.circle(img, (cuboidProj[i][0], cuboidProj[i][1]), 5, (255, 0, 0), 5)
		linePoints = [[cuboidProj[0], cuboidProj[1]],
					  [cuboidProj[1], cuboidProj[2]],
					  [cuboidProj[2], cuboidProj[3]],
					  [cuboidProj[3], cuboidProj[0]],

					  [cuboidProj[4], cuboidProj[5]],
					  [cuboidProj[5], cuboidProj[6]],
					  [cuboidProj[6], cuboidProj[7]],
					  [cuboidProj[7], cuboidProj[4]],

					  [cuboidProj[0], cuboidProj[4]],
					  [cuboidProj[1], cuboidProj[5]],
					  [cuboidProj[2], cuboidProj[6]],
					  [cuboidProj[3], cuboidProj[7]],
					 ]
		for i in range(4):
			cv2.line(img, (linePoints[i][0][0], linePoints[i][0][1]), (linePoints[i][1][0], linePoints[i][1][1]), (0, 255, 0), 2)
		for i in range(4, 8):
			cv2.line(img, (linePoints[i][0][0], linePoints[i][0][1]), (linePoints[i][1][0], linePoints[i][1][1]), (0, 0, 255), 2)
		for i in range(8, 12):
			cv2.line(img, (linePoints[i][0][0], linePoints[i][0][1]), (linePoints[i][1][0], linePoints[i][1][1]), (0, 255, 255), 2)
		offset = 10
		for i in range(len(cuboidProj)):
			cv2.putText(img, str(i), (cuboidProj[i][0] + offset, cuboidProj[i][1] + offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
		return img

	###########################################################
	# CALLBACKS ###############################################
	###########################################################
	def cameraInfoRGB_callback(self, data):
		fx = data.K[0]	#925.112183
		fy = data.K[4]	#925.379517
		cx = data.K[2]	#647.22644
		cy = data.K[5]	#357.068359
		self.intrinsics = [fx, fy, cx, cy]
		#print self.intrinsics
		#print self.fx, self.fy, self.cx, self.cy

		# Calculate intrinsic parameters for resized image
		self.set_resized_intrinsics()

		# Store camera-parameters in dict
		self.camera_settings_rgb = self.get_camera_dict(data.width, data.height, self.intrinsics, "RealsenseD435_RGB", 69.40) # FOV Read out of camera and also from datasheet
		self.camera_settings_rgb_resized = self.get_camera_dict(self.imgOutputSize, self.imgOutputSize, self.intrinsics_resized, "RealsenseD435_RGB_resized", 42.50) # FOV = Vertical FOV fromd atasheet

		# Unregister from Camera-Info
		self.rgb_info_sub.unregister()

	def cameraInfoD_callback(self, data):
		fx = data.K[0]
		fy = data.K[4]
		cx = data.K[2]
		cy = data.K[5]

		# Store camera-parameters in dict
		self.camera_settings_depth = self.get_camera_dict(data.width, data.height, [fx, fy, cx, cy], "RealsenseD435_Depth", 69.40)	# FOV as of color camera because of alignment

		# Unregister from Camera-Info
		self.d_info_sub.unregister()

	def rgb_image_callback(self, data):
		try:
			cv_rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			# Store image locally and resize it
			self.rgb_img = cv_rgb_image.copy()
			self.rgb_resize()
		except CvBridgeError as e:
			print(e)

	def d_image_callback(self, data):
		try:
			cv_depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			# Store image locally
			self.d_img = cv_depth_image.copy()
		except CvBridgeError as e:
			print(e)

	def pose_callback(self, data):
		self.goals = data

	###########################################################
	# HELPERS #################################################
	###########################################################
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
		#cv2.imshow("Image-Stream", output)
		#cv2.waitKey(1)


	# Set the intrinsic camera-parameters according to parameters of rgb-image and resize-factor
	def set_resized_intrinsics(self):
		# Scale the paramters because of 720 to 400px shrinking
		intrinsics_resized = [i/self.img_scaleFac for i in self.intrinsics]
		# move cx to locate it at center again
		intrinsics_resized[2] = intrinsics_resized[2] - self.resized_img_horizontalStart
		self.intrinsics_resized = intrinsics_resized

	# Check if all cuboid-poses are visible in the resized image
	def check_obj_in_img(self):
		positions = self.calculate_projection(self.get_cuboid_transforms(), self.intrinsics_resized)
		for i in range(len(positions)-1):	# -1 because center can not be out of image
			if positions[i][0] > self.imgOutputSize or positions[i][0] < 0 or positions[i][1] > self.imgOutputSize or positions[i][1] < 0:
				return False
		return True

	# Get permuted transformation matrix for fat-dataset-format
	def get_permuted_matrix_from_pose(self, pose):
		quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
		M_rot = tf.transformations.quaternion_matrix(quat)
		M_res = np.delete(M_rot, 3, 1)	# Delete last column
		M_res = np.delete(M_res, 3, 0)	# Delete last row
		P_inv = np.linalg.inv([[0, 0, 1],[1, 0, 0],[0, -1, 0]])
		M_out = np.matmul(M_res, P_inv).transpose()

		transl = [pose.position.x*100, pose.position.y*100, pose.position.z*100, 1]
		M_out = np.c_[M_out, np.zeros(3)]		# Add empty column as last column
		M_out = np.append(M_out, values=transl)	# Add translation as last row
		return M_out.tolist()

	# Calculate projection of cuboidPoses on image plane
	def calculate_projection(self, cuboidPoses, intrinsics):
		cuboidProj = []
		for i in range(len(cuboidPoses.poses)):
			p = [0, 0]
			p[0] = intrinsics[2] + 1/cuboidPoses.poses[i].position.z * (intrinsics[0] * cuboidPoses.poses[i].position.x)
			p[1] = intrinsics[3] + 1/cuboidPoses.poses[i].position.z * (intrinsics[1] * cuboidPoses.poses[i].position.y)
			cuboidProj.append(p)
		return cuboidProj

	###########################################################
	# Functions to get dicts ##################################
	###########################################################
	# Store camera-parameters in dict
	def get_camera_dict(self, w, h, intrinsics, name, fov):
		captured_image_size = {"width": w, "height": h}
		intrinsic_settings = {"resX": w, "resY": h, "fx": intrinsics[0], "fy": intrinsics[1], "cx": intrinsics[2], "cy": intrinsics[3], "s": 0}
		cam_settings = {"name": name, "horizontal_fov": fov, "intrinsic_settings": intrinsic_settings, "captured_image_size": captured_image_size}
		return cam_settings

	# Store all image-parameters in dict
	def get_data_dict(self, intrinsics):
		''' # DEBUG
		baseToCam = Pose()
		baseToCam.position.x = 0
		baseToCam.position.y = 1
		baseToCam.position.z = 0.5
		baseToCam.orientation.x = 0.707
		baseToCam.orientation.y = -0.707
		baseToCam.orientation.z = 0
		baseToCam.orientation.w = 1
		camToObj = baseToCam'''

		''' # DEBUG LOAD
		with open(str(self.path) + '000000.json') as json_file:  
			data = json.load(json_file)
		cd = data["camera_data"]
		print cd["location_worldframe"]'''

		# Get necessary transformations
		baseToCam = self.get_transform('/base_link', '/camera_color_optical_frame')
		camToObj = self.get_transform('/camera_color_optical_frame', '/object')

		# Camera Data
		location_worldframe = [baseToCam.position.x*100, baseToCam.position.y*100, baseToCam.position.z*100]
		quaternion_xyzw_worldframe = [baseToCam.orientation.x, baseToCam.orientation.y, baseToCam.orientation.z, baseToCam.orientation.w]
		camera_data = {'location_worldframe': location_worldframe, 'quaternion_xyzw_worldframe': quaternion_xyzw_worldframe}

		# Get Cuboids
		cuboidPoses = self.get_cuboid_transforms()
		cuboidProj = self.calculate_projection(cuboidPoses, intrinsics)

		# Object Data
		location = [camToObj.position.x*100, camToObj.position.y*100, camToObj.position.z*100]
		quaternion_xyzw = [camToObj.orientation.x, camToObj.orientation.y, camToObj.orientation.z, camToObj.orientation.w]
		pose_transform = self.get_permuted_matrix_from_pose(camToObj)
		
		cuboid_centroid = [cuboidPoses.poses[8].position.x, cuboidPoses.poses[8].position.y, cuboidPoses.poses[8].position.z]
		projected_cuboid_centroid = [cuboidProj[8][0], cuboidProj[8][1]]
		bounding_box = {"top_left": "NaN", "bottom_right": "NaN"}
		cuboid = []
		for i in range(len(cuboidPoses.poses) - 1):
			cuboid.append([cuboidPoses.poses[i].position.x*100, cuboidPoses.poses[i].position.y*100, cuboidPoses.poses[i].position.z*100])
		projected_cuboid = []
		for i in range(len(cuboidProj) - 1):
			projected_cuboid.append([cuboidProj[i][0], cuboidProj[i][1]])
		objects = {"class": self.objectName, "instance_id": 0, "visibility": 1, "location": location, "quaternion_xyzw": quaternion_xyzw,
					"pose_transform": pose_transform, "cuboid_centroid": cuboid_centroid, "projected_cuboid_centroid": projected_cuboid_centroid,
					"bounding_box": bounding_box, "cuboid": cuboid, "projected_cuboid": projected_cuboid}

		data = {'camera_data': camera_data, 'objects': objects}
		return data, cuboidProj

	###########################################################
	# TRANSFORMATIONS #########################################
	###########################################################
	# Convert list to pose-object
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

	# Convert pose-object to list
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

	# Get transformation from one frame to another
	def get_transform(self, fromFrame, toFrame):
		try:
			now = rospy.Time.now()
			self.listener.waitForTransform(fromFrame, toFrame, now, rospy.Duration(4.0))
			(trans, rot) = self.listener.lookupTransform(fromFrame, toFrame, now)
			return self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)
			return None

	# All transformations
	def get_transformations(self):
		baseObjPose = self.get_transform('/base_link', '/object')
		baseCamPose = self.get_transform('/base_link', '/camera_color_optical_frame')
		camObjPose = self.get_transform('/camera_color_optical_frame', '/object')
		objCamPose = self.get_transform('/object', '/camera_color_optical_frame')
		baseToolPose = self.get_transform('/base_link', '/tool0_controller')
		return baseObjPose, baseCamPose, camObjPose, objCamPose, baseToolPose

	# From camera to cuboid-poses
	def get_cuboid_transforms(self):
		cuboidPoses = PoseArray()
		now = rospy.Time.now()
		for i in range(9):
			cuboidPoses.poses.append(self.get_transform('/camera_color_optical_frame', '/c_'+str(i)))
		return cuboidPoses

	###########################################################
	# ROBOT-RELATED ###########################################
	###########################################################
	# Drive robot to given pose-id
	def drive_to_pose(self, id):
		self.ur5.execute_move(self.goals.poses[id])

	# Move the robot, check if object is not cropped in image and store state
	def move_check_store(self, joint, rot):
		counter = 0
		while True:
			self.ur5.move_joint(joint, rot)
			if self.check_obj_in_img() == True:
				break
			if counter > 3:
				print("Can't correct cropped object!")
				return
			rot = rot / 2 - rot
			print_debug("Object cropped - correcting...")
			counter = counter + 1
		self.store_state()

	# Make random moves with last axes
	def move_random(self):
		# Sample random offsets
		rotateUp = random.uniform(self.rotateUpRMin, self.rotateUpRMax)
		rotateDown = random.uniform(-self.rotateUpRMin, -self.rotateUpRMax)
		rotateTiltL = random.uniform(self.rotateTiltRMin, self.rotateTiltRMax)
		rotateTiltR = random.uniform(-self.rotateTiltRMin, -self.rotateTiltRMax)

		# Execute offsets
		print_debug("RotUp" + str(rotateUp))
		self.move_check_store(4, rotateUp)
		print_debug("RotD" + str(rotateDown))
		self.move_check_store(4, rotateDown - rotateUp)
		print_debug("TiltL" + str(rotateTiltL))
		self.move_check_store(3, rotateTiltL)
		print_debug("TiltR" + str(rotateTiltR))
		self.move_check_store(3, rotateTiltR - rotateTiltL)

	# Drive to the goals and make random moves
	def capture(self, startID):
		for i in range(startID, len(self.goals.poses)):
			self.actPoseID = i		
			self.ur5.execute_move(self.goals.poses[i])		# Move to base-point
			self.move_check_store(0, 0)
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(self.rotateRMin, self.rotateRMax)
			print_debug("Rotating1 " + str(rotateRand))
			self.move_check_store(5, rotateRand)			# Rotate the EEF
			self.move_random()								# Make random moves
			self.ur5.execute_move(self.goals.poses[i])		# Move back to base-point

			rotateRand = random.uniform(-self.rotateRMin, -self.rotateRMax)
			print_debug("Rotating2 " + str(rotateRand))
			self.move_check_store(5, rotateRand)			# Rotate the EEF
			self.move_random()								# Make random moves

		''' # DEBUG
		i = 0
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

	###########################################################
	# STORE DATA ##############################################
	###########################################################
	# Write data to json-file (formatted)
	def write_json(self, data, path, filename):
		dump = json.dumps(data, sort_keys=False, indent=4)
		new_data = re.sub('\n +', lambda match: '\n' + '\t' * (len(match.group().strip('\n')) / 3), dump)
		print >> open(str(path) + str(filename), 'w'), new_data

	# Collect all info for camera-setting-file and write it
	def write_cam_settings(self):
		while True:
			if (self.camera_settings_rgb is not None and self.camera_settings_depth is not None and self.camera_settings_rgb_resized is not None):
				break
			print ("Waiting for camera-settings to arrive...")
			rospy.sleep(0.5)
		data = {"camera_settings": [self.camera_settings_rgb, self.camera_settings_depth]}
		self.write_json(data, self.pathFullRes, "_camera_settings.json")
		data = {"camera_settings": [self.camera_settings_rgb_resized]}
		self.write_json(data, self.path, "_camera_settings.json")

	# Collect all info for object-setting-file and write it
	def write_scene_settings(self):
		baseObjPose = self.get_transform('/base_link', '/object')

		# INFO:
		# Segmentation-id = 255 since only 'Single'-images are produced (see FAT readme)
		# Cuboid-Dimensions in cm extracted from dataset-synthesizer json-file
		fixed_model_transform = self.get_permuted_matrix_from_pose(baseObjPose)
		exported_objects = {"class": self.objectName, "segmentation_class_id": 255, "segmentation_instance_id": 255, "fixed_model_transform": fixed_model_transform, "cuboid_dimensions": [19.8, 90.0, 177.0]}
		data = {"exported_object_classes": [self.objectName], "exported_objects": [exported_objects]}
		self.write_json(data, self.path, "_object_settings.json")
		self.write_json(data, self.pathFullRes, "_object_settings.json")

	# Store images and poses at actual position to json-files
	def store_state(self):
		# Calculate pose-info for user output
		if self.lastPoseID == self.actPoseID:
			self.actStorage = self.actStorage + 1
		else:
			self.lastPoseID = self.actPoseID
			self.actStorage = 0
		poseInfo = str(self.actPoseID) + "_" + str(self.actStorage)
		fileName = '{0:06d}'.format(self.actStoreID)
		self.actStoreID = self.actStoreID + 1

		# Make sure to refresh data -> Set None and wait until new images arrive
		self.d_img = None
		self.rgb_img = None
		self.rgb_img_resized = None
		counter = 0
		while self.d_img is None or self.rgb_img is None or self.rgb_img_resized is None:
			counter = counter + 1
			if counter > 200:
				print "No images arrived! Continuing with next pose..."
			rospy.sleep(0.05)

		# Store Images locally
		d_img = self.d_img.copy()
		rgb_img = self.rgb_img.copy()
		rgb_img_resized = self.rgb_img_resized.copy()

		# Save OpenCV2 images
		cv2.imwrite(str(self.pathFullRes) + str(fileName) + "_rgb.png", rgb_img)
		cv2.imwrite(str(self.pathFullRes) + str(fileName) + "_d.png", d_img*255)	# *255 to rescale from 0-1 to 0-255
		cv2.imwrite(str(self.path) + str(fileName) + ".png", rgb_img_resized)

		# Store Depth-Image as CSV-File
		with open(str(self.pathFullRes) + str(fileName) + "_d.csv", "wb") as f:
			writer = csv.writer(f, delimiter=";")
			for line in d_img:
				writer.writerow(line)

		# Store Depth-Image as Point-Cloud
		if storePC == True:
			f1 = open(str(self.pathFullRes) + str(fileName) + "pc.ply", "w")
			f1.write("ply\nformat ascii 1.0\nelement vertex 921600\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
			for row in range(len(d_img)):			#1280
				for col in range(len(d_img[0])):	#720
					f1.write(str(float(row) / 1000.) + " " + str(float(col) / 1000.) + " " + str(float(d_img[row][col]) / 1000.) + "\n")
			f1.close()

		# Store information-file
		data = self.get_data_dict(self.intrinsics)
		self.write_json(data, self.pathFullRes, str(fileName) + ".json")
		data, cuboidPoses = self.get_data_dict(self.intrinsics_resized)
		self.write_json(data, self.path, str(fileName) + ".json")

		# Draw cuboids on image and show image to user
		output = self.draw_cuboids(self.rgb_img_resized, cuboidPoses)
		cv2.imshow("Image-Stored", output)
		cv2.waitKey(1)

		# Store all camera-poses
		actCamPose = self.poseToList(self.get_transform('/base_link', '/camera_color_optical_frame'))
		self.camPoses.append(actCamPose)
		data = self.camPoses
		self.write_json(data, self.pathFullRes, "_camera_poses.json")

		print "Stored " + str(poseInfo) + " as " + str(fileName)

		''' # DEBUG
		baseObjPose, baseCamPose, camObjPose, objCamPose, baseToolPose = self.get_transformations()
		data = {"camToObj_xyz_xyzw": self.poseToList(camObjPose),
				"objToCam_xyz_xyzw": self.poseToList(objCamPose),
				"baseToObj_xyz_xyzw": self.poseToList(baseObjPose),
				"baseTotool0_controller_xyz_xyzw": self.poseToList(baseToolPose),
				"baseToCam_xyz_xyzw": self.poseToList(baseCamPose),}
		self.write_json(data, str(fileName) + "_poses.json")'''

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init node
	rospy.init_node('data_capture', anonymous=True, disable_signals=True)

	# Check arguments and store path
	if len(args) < 2:
		print "Please specify folder to store files!"
		return
	else:
		path = rospy.get_param("path_to_store")
		pathFullRes = path + str(args[1]) + "_full_res" + "/"
		path = path + str(args[1]) + "/"
		if not os.path.exists(path):
			os.makedirs(path)
			os.makedirs(pathFullRes)
		else:
			rospy.logwarn("You are writing to an existing folder!")
	
	# Init dataCapture-Module and start capturing
	dc = dataCapture(path, pathFullRes)
	startID = 0
	if len(args) == 3:
		startID = int(args[2])
		print "Starting at pose no. " + str(startID)
	rospy.logwarn("Don't forget to put away the marker!")

	''' # DEBUG
	while True:
		inp = raw_input("y to Store, e to Exit, n to continue: ")[0]
		if inp == 'y':
			dc.store_state()
		elif inp == 'e':
			return'''

	dc.capture(startID)

if __name__ == '__main__':
	main(sys.argv)
