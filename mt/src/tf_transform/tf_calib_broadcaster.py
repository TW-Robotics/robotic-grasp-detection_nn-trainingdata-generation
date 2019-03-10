#!/usr/bin/env python  
import rospy
import tf
import sys
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray

''' Subscribe to transformation between object and camera and 
	broadcast it to tf so other transformations can be calculated'''

marker_poses = FiducialTransformArray()

def obj_pose_callback(data):
	global marker_poses
	marker_poses = data

def main(args):
	# Init Node
	rospy.init_node('tf_broadcaster', disable_signals=True)

	# Init subscriber to Pose of object relative to camera
	rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, obj_pose_callback, queue_size=1)

	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Send transformation to tf
		for t in marker_poses.transforms:
			br.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
							 (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
							 rospy.Time(0),
							 "camera_marker",
							 "camera_color_optical_frame")
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)