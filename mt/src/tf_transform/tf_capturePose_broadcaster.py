#!/usr/bin/env python  
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

capturePoses = PoseArray()
objToBasePose = Pose()

def pose_callback(data):
	global capturePoses
	capturePoses = data

def objToBasePose_callback(data):
	global objToBasePose
	objToBasePose = data

def main(args):
	# Init Node
	rospy.init_node('tf_capturePose_broadcaster', disable_signals=True)

	# Init subscriber to Pose of object relative to camera
	rospy.Subscriber("/capturePoses", PoseArray, pose_callback, queue_size=1)
	rospy.Subscriber("/tf_objToBase", Pose, objToBasePose_callback, queue_size=1)
	# TODO Subscribe to objectPose

	#v0 = np.array([0.3, 0.6, -0.15])

	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Send transformation to tf
		#br.sendTransform((v0[0], v0[1], v0[2]), (0, 0, 0, 1), rospy.Time.now(), "object", "world")
		for i in range(len(capturePoses.poses)):
			try:
				br.sendTransform((capturePoses.poses[i].position.x , capturePoses.poses[i].position.y , capturePoses.poses[i].position.z ),
								 (capturePoses.poses[i].orientation.x, capturePoses.poses[i].orientation.y, capturePoses.poses[i].orientation.z, capturePoses.poses[i].orientation.w),
								 rospy.Time.now(),
								 "p_" + str(i),
								 "object")
			except:
				rospy.loginfo("Warning!")
				continue
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)