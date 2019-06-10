#!/usr/bin/env python  
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge, CvBridgeError

debug = True

class graspPoint_broadcaster():
	def __init__(self):
		self.bridge = CvBridge()
		self.br = tf.TransformBroadcaster()
		self.tfListener = tf.TransformListener()

		# Init subscriber
		rospy.Subscriber("/dope/pose_carrier_empty", PoseStamped, self.pose_callback, queue_size=1)					# Pose transform camera to grasp-point
		rospy.Subscriber("/dope/rgb_points", Image, self.rgb_image_callback)									# RGB-Image
		self.pose_update_pub = rospy.Publisher("/dope/pose_update", Bool, queue_size=10)

	def rgb_image_callback(self, data):
		rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Visualization
		cv2.imshow("Dope output", rgb_img)
		cv2.waitKey(1)

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

	# Broadcast the grasp-point to tf
	def pose_callback(self, objectPose):
		now = rospy.Time.now()
		# Broadcast camera -> object
		self.br.sendTransform((objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z),
							 (objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z, objectPose.pose.orientation.w),
							 now,
							 "dope_object_pose",
							 "camera_color_optical_frame")

		# Lookup base -> object
		try:
			(trans, rot) = self.tfListener.lookupTransform('/base_link', '/dope_grasp_point', now)
			objectPose = self.listToPose(trans, rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)

		# Broadcast base -> object (So object does not move if camera moves)
		self.br.sendTransform((objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z),
							 (objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z, objectPose.pose.orientation.w),
							 now,
							 "dope_object_pose",
							 "base_link")

		# Inform other nodes that object pose has been updated
		self.pose_update_pub.publish(Bool(True))

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	# Init Node
	rospy.init_node('tf_dope_pose_broadcaster', disable_signals=True)

	graspPoint_br = graspPoint_broadcaster()

	rospy.loginfo("TF-Grasp-Point-Broadcaster sucessfully launched!\nBroadcasting to tf...")

	rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
