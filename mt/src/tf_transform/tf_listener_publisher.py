#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose
import sys
from math import pi

# Convert lists to pose-obect so a standard pose message can be published
def listToPose(trans, rot):
	pose = Pose()
	pose.position.x = trans[0]
	pose.position.y = trans[1]
	pose.position.z = trans[2]
	pose.orientation.x = rot[0]
	pose.orientation.y = rot[1]
	pose.orientation.z = rot[2]
	pose.orientation.w = rot[3]
	return pose

def main(args):
	# Init Node
	rospy.init_node('tf_listener_publisher', disable_signals=True)
	
	# Init Listener for tf-transformation
	listener = tf.TransformListener()

	# Init Publishers
	baseToObjPub = rospy.Publisher("/tf_baseToObj", Pose, queue_size=1)
	objToCamPub = rospy.Publisher("/tf_objToCam", Pose, queue_size=1)
	objImgCenterToCamPub = rospy.Publisher("/tf_objImgCenterToCam", Pose, queue_size=1)
	rospy.sleep(1)

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			# Get transformation and publish it rearranged to a list
			(trans, rot) = listener.lookupTransform('/base_link', '/object', rospy.Time(0))
			(trans1, rot1) = listener.lookupTransform('/object', '/camera_color_optical_frame', rospy.Time(0))	# transform from object to camera (anders als in Doku)
			(trans2, rot2) = listener.lookupTransform('/object_img_center', '/camera_color_optical_frame', rospy.Time(0))	# transform from object to camera (anders als in Doku)
			baseToObjPub.publish(listToPose(trans, rot))
			objToCamPub.publish(listToPose(trans1, rot1))
			objImgCenterToCamPub.publish(listToPose(trans2, rot2))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("Warning!")
			continue
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)