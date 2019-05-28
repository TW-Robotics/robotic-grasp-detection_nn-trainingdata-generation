#!/usr/bin/env python  
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

debug = True

class marker_broadcaster():
	def __init__(self):

		self.markerCenter = Point()
		self.dImage = Image()

		# Instantiate CvBridge
		self.bridge = CvBridge()

		# Init subscriber
		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.marker_callback, queue_size=1)
		rospy.Subscriber("/fiducial_vertices", FiducialArray, self.marker_vert_callback, queue_size=1)				# Marker-Corners
		rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)									# RGB-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.d_image_callback)					# Depth-Image

	# Broadcast the refined, mean gt-pose (final gt-pose)
	def marker_callback(self, marker_poses):
		br = tf.TransformBroadcaster()
		
		if len(marker_poses.transforms) > 0:
			if self.markerCenter.z != 0:
				#print "Marker_Det: " + str(marker_poses.transforms[0].transform.translation.z)
				#print "Refinement: " + str(float(self.markerCenter.z) / 1000.)
				print_debug("depth refined")
				marker_poses.transforms[0].transform.translation.z = float(self.markerCenter.z) / 1000.
			#else:
				#print "Marker_Det: " + str(marker_poses.transforms[0].transform.translation.z)
				#print_debug("not refined")
			#print marker_poses.transforms[0].transform.translation.z
			br.sendTransform((marker_poses.transforms[0].transform.translation.x, marker_poses.transforms[0].transform.translation.y, marker_poses.transforms[0].transform.translation.z),
							 (marker_poses.transforms[0].transform.rotation.x, marker_poses.transforms[0].transform.rotation.y, marker_poses.transforms[0].transform.rotation.z, marker_poses.transforms[0].transform.rotation.w),
							 rospy.Time.now(),
							 "m_det",
							 "camera_color_optical_frame")
			# Set zero to make sure it is refreshed
			self.markerCenter.z = 0


	'''# Broadcast the refined, mean gt-pose (final gt-pose)
	def marker_callback(self, marker_poses):
		br = tf.TransformBroadcaster()
		
		if len(marker_poses.transforms) > 0:
			if self.markerCenter.z != 0:
				print "Marker_Det: " + str(marker_poses.transforms[0].transform.translation.z)
				print "Refinement: " + str(float(self.markerCenter.z) / 1000.)
				print_debug("depth refined")
				#marker_poses.transforms[0].transform.translation.z = float(self.markerCenter.z) / 1000.
				br.sendTransform((marker_poses.transforms[0].transform.translation.x, marker_poses.transforms[0].transform.translation.y, float(self.markerCenter.z) / 1000.),
				 (marker_poses.transforms[0].transform.rotation.x, marker_poses.transforms[0].transform.rotation.y, marker_poses.transforms[0].transform.rotation.z, marker_poses.transforms[0].transform.rotation.w),
				 rospy.Time.now(),
				 "m_det_refined",
				 "camera_color_optical_frame")
				try:
					now = rospy.Time.now()
					tf.TransformListener().waitForTransform("base_link", "m_det_refined", now, rospy.Duration(4.0))
					(trans, rot) = tf.TransformListener().lookupTransform("base_link", "m_det_refined", now)
					print "Refinement: " + str(trans) + str(rot)
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
					rospy.logerr(e)
			else:
				print "Marker_Det: " + str(marker_poses.transforms[0].transform.translation.z)
				#print_debug("not refined")
			#print marker_poses.transforms[0].transform.translation.z
			br.sendTransform((marker_poses.transforms[0].transform.translation.x, marker_poses.transforms[0].transform.translation.y, marker_poses.transforms[0].transform.translation.z),
							 (marker_poses.transforms[0].transform.rotation.x, marker_poses.transforms[0].transform.rotation.y, marker_poses.transforms[0].transform.rotation.z, marker_poses.transforms[0].transform.rotation.w),
							 rospy.Time.now(),
							 "m_det_notRefined",
							 "camera_color_optical_frame")
			rospy.sleep(1)
			try:
				now = rospy.Time.now()
				tf.TransformListener().waitForTransform("base_link", "m_det_notRefined", now, rospy.Duration(4.0))
				(trans, rot) = tf.TransformListener().lookupTransform("base_link", "m_det_notRefined", now)
				print "Marker_Det: " + str(trans) + str(rot)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr(e)
			# Set zero to make sure it is refreshed
			self.markerCenter.z = 0'''


	# Calculate center of marker
	def marker_vert_callback(self, data):
		# If at least one marker could be found, calculate its center
		if len(data.fiducials) > 0:
			vert = data.fiducials[0]
			self.markerCenter.x = int((vert.x0+vert.x1+vert.x2+vert.x3)/4)
			self.markerCenter.y = int((vert.y0+vert.y1+vert.y2+vert.y3)/4)

	def rgb_image_callback(self, data):
		rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		''' # Visualization
		cv2.circle(rgb_img ,(int(self.markerCenter.x), int(self.markerCenter.y)),2,(0,0,255),3)
		cv2.imshow("Img", rgb_img)
		cv2.waitKey(1)'''

	def d_image_callback(self, data):
		self.d_img = self.bridge.imgmsg_to_cv2(data, "16UC1")

		# If a depth-image arrives, calculate the depth-value of the marker's center and publish it
		self.markerCenter.z = self.d_img[int(self.markerCenter.y)][int(self.markerCenter.x)]
		#print self.markerCenter.z

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init Node
	rospy.init_node('tf_marker_broadcaster', disable_signals=True)

	marker_br = marker_broadcaster()

	rospy.loginfo("TF-Marker-Broadcaster sucessfully launched!\nRefining marker-pose and broadcasting to tf...")

	rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
