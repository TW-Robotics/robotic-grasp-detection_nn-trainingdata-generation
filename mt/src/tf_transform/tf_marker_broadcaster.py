#!/usr/bin/env python  
import rospy
import tf
import sys
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray

''' Subscribes to transformations from markers to camera calculated by aruco_detect
	and broadcasts these transforms in the tf-tree. Camera-coordinate-frame at br.sendTransform
	has to be changed if name is different '''

# Store detected markers in global array
marker_poses = FiducialTransformArray()

# Callback for markers
def obj_pose_callback(data):
	global marker_poses
	marker_poses = data

def main(args):
	# Init Node
	rospy.init_node('tf_broadcaster', disable_signals=True)

	# Init subscriber to transformation camera->marker
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
							 rospy.Time.now(),
							 "marker_" + str(t.fiducial_id),
							 "camera_color_optical_frame")
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)