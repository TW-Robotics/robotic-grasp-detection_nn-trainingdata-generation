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

		# Init subscriber
		rospy.Subscriber("/dope/pose_carrier_empty", PoseStamped, self.pose_callback, queue_size=1)					# Pose transform camera to grasp-point
		rospy.Subscriber("/dope/rgb_points", Image, self.rgb_image_callback)									# RGB-Image
		self.pub = rospy.Publisher("/dope/pose_update", Bool, queue_size=10)

	def rgb_image_callback(self, data):
		rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Visualization
		cv2.imshow("Dope output", rgb_img)
		cv2.waitKey(1)

	# Broadcast the grasp-point to tf
	def pose_callback(self, objectPose):
		self.br.sendTransform((objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z),
							 (objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z, objectPose.pose.orientation.w),
							 rospy.Time.now(),
							 "dope_object_pose",
							 "camera_color_optical_frame")
		self.pub.publish(Bool(True))

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
