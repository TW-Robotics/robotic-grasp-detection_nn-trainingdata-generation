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


if __name__ == '__main__':
	rospy.init_node("tf_echo")
	listener = tf.TransformListener()

	rate = rospy.Rate(10.)
	while not rospy.is_shutdown():
		try:
			#now = rospy.Time.now()
			#listener.waitForTransform("base_link", "m_det", now, rospy.Duration(4.0))
			(trans, rot) = listener.lookupTransform("base_link", "m_det", rospy.Time(0))
			print trans, rot
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr(e)
			continue
		rate.sleep()