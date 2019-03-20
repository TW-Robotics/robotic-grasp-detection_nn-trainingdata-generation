#!/usr/bin/env python  
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import PoseArray

gtPoses = PoseArray()

def pose_callback(data):
	global gtPoses
	gtPoses = data

def main(args):
	# Init Node
	rospy.init_node('tf_gtPose_broadcaster', disable_signals=True)

	# Init subscriber to Pose of object relative to camera
	rospy.Subscriber("/gtPoses", PoseArray, pose_callback, queue_size=1)
	# TODO Subscribe to objectPose

	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Send transformation to tf
		for i in range(len(gtPoses.poses)):
			try:
				br.sendTransform((gtPoses.poses[i].position.x, gtPoses.poses[i].position.y, gtPoses.poses[i].position.z),
								 (gtPoses.poses[i].orientation.x, gtPoses.poses[i].orientation.y, gtPoses.poses[i].orientation.z, gtPoses.poses[i].orientation.w),
								 rospy.Time.now(),
								 "mp_" + str(i),
								 "base_link")
			except:
				rospy.loginfo("Warning!")
				continue
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)