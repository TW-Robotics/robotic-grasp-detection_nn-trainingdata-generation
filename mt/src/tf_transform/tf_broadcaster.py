#!/usr/bin/env python  
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Point

marker_center = Point()
debug = True

# Broadcast the refined, mean gt-pose (final gt-pose)
def marker_callback(marker_poses):
	global marker_center
	br = tf.TransformBroadcaster()

	if marker_center.z != 0:
		print_debug("depth refined")
		marker_poses.transforms.transform.translation.z = marker_center.z
	else:
		print_debug("not refined")
	br.sendTransform((marker_poses.transforms.transform.translation.x, marker_poses.transforms.transform.translation.y, marker_poses.transforms.transform.translation.z),
					 (marker_poses.transforms.transform.rotation.x, marker_poses.transforms.transform.rotation.y, marker_poses.transforms.transform.rotation.z, marker_poses.transforms.transform.rotation.w),
					 rospy.Time.now(),
					 "marker_" + str(marker_poses.transforms.fiducial_id),
					 "camera_color_optical_frame")

	# Set zero to make sure it is refreshed
	marker_center.z = 0

# Get center of marker (depth-refinement)
def markerCenter_callback(center):
	global marker_center
	marker_center = center

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init Node
	rospy.init_node('tf_broadcaster', disable_signals=True)

	# Init subscriber
	rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, marker_callback, queue_size=1)
	rospy.Subscriber("/markerCenter", Point, markerCenter_callback, queue_size=1)

	rospy.loginfo("TF Broadcaster sucessfully launched!\nBroadcasting to tf...")

	rospy.spin()

if __name__ == '__main__':
    main(sys.argv)