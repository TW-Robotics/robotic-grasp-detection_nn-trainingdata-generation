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

	minObjError = 1.0
	mID = 240
	marker240toObj = [0.160, -0.309, 0.230, -0.016, -0.012, -0.382, 0.924]
	marker241toObj = [0.2, 0, 0, 0, 0, 0, 1]
	marker242toObj = [-0.307, 0.162, 0.201, 0.004, -0.001, 0.923, -0.385]
	marker243toObj = [-0.303, 0.167, 0.204, -0.012, 0.007, 0.385, 0.923]
	marker244toObj = [-0.286, 0.175, 0.204, 0.021, 0.021, -0.128, 0.991]
	mToObj = {240: marker240toObj, 241: marker241toObj, 242: marker242toObj, 243: marker243toObj, 244: marker244toObj}

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Send transformation to tf
		for t in marker_poses.transforms:
			br.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
							 (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
							 rospy.Time.now(),
							 "marker_" + str(t.fiducial_id),
							 "camera")
			if t.object_error < minObjError:
				minObjError = t.object_error
				mID = t.fiducial_id
			#print mID
		#br.sendTransform((0, 0, 0.2), (0, 0, 0, 1), rospy.Time.now(), "object", "marker_241")
		br.sendTransform((mToObj[mID][0], mToObj[mID][1], mToObj[mID][2]), (mToObj[mID][3], mToObj[mID][4], mToObj[mID][5], mToObj[mID][6]), rospy.Time.now(), "object", "marker_" + str(mID))
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)