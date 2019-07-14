#!/usr/bin/env python
import numpy as np
import sys
import select
import rospy
import tf
import math
import collections
import csv
import time
from math import pi
import argparse

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# IF IMPORT NOT WORKING: sys.path.append('/home/johannes/git/mt/mt/src/grasp_object/')
from mir_control import mir_control
from mission_control import mission_control
from grasp_control import grasp_control

debug = False
if rospy.get_param("print_debug") == True:
	print "Debug-Mode ON"
	debug = True		# Print Debug-Messages

# Print debug messages
def print_debug(dStr):
	global debug
	if debug == True:
		print dStr

def main(args):
	parser = argparse.ArgumentParser(description='Transport Object')
	parser.add_argument('pickUpGoal', metavar='pickUpGoal', type=str, help='Name of goal to pick up carrier')
	parser.add_argument('putDownGoal', metavar='putDownGoal', type=str, help='Name of goal to put down carrier')
	pickUpGoal = mission_control.goal(parser.parse_args().pickUpGoal)
	putDownGoal = mission_control.goal(parser.parse_args().putDownGoal)
	
	# Init node
	rospy.init_node('transport_object', anonymous=True, disable_signals=True)
	rospy.loginfo("Make sure correct camera intrinsics are set in yaml file!")

	mir = mir_control.mirControler()
	grasper = grasp_control.grasp_process()

	while (True):
		grasper.reset_poses()
		inp = raw_input("p to publish image, a for search+grasp, b for search+put, \
						 s to store, u to unstore, \
						 d to drive to pickUpGoal, e to drive to putDownGoal: ")[0]
		
		if inp == 'd':
			grasper.mir.moveToGoal(pickUpGoal.posx, pickUpGoal.posy, pickUpGoal.rz)
			while grasper.mir.isAtGoal(0.2, 0.1) == False:
				print "moving..."
				rospy.sleep(1)

		if inp == 'e':
			grasper.mir.moveToGoal(putDownGoal.posx, putDownGoal.posy, putDownGoal.rz)
			while grasper.mir.isAtGoal(0.2, 0.1) == False:
				print "moving..."
				rospy.sleep(1)

		if inp == 's':
			grasper.store()

		if inp == 'u':
			grasper.unstore()

		if inp == 'p':
			grasper.publish_image()

		elif inp == 'a':
			grasper.hasGraspedPub.publish(Bool(True))
			grasper.hasPutPub.publish(Bool(True))
			grasper.ur5.removeAttachedObject()
			grasper.ur5.scene.remove_world_object()
			grasper.ur5.moveToSearchPose(pickUpGoal.orientation)
			rospy.sleep(0.1)	# wait to arrive at position
			grasper.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while grasper.poseIsUpdated_carrier == False and time.time() <= timeout:
				rospy.sleep(0.05)
			posID = 0
			while grasper.poseIsUpdated_carrier == False:		# if the object has been found, this is going to be True and outer loop is exited
				if grasper.ur5.searchObject(posID) == False:
					print "No object found!"
					break
				posID = posID + 1
				rospy.sleep(0.1) 	# wait to arrive at position
				grasper.publish_image()
				timeout = time.time() + 1.5   # 1.5 seconds from now
				# Wait until updated pose arrives or timeout occurs (pose not visible)
				while grasper.poseIsUpdated_carrier == False:
					rospy.sleep(0.05)
					if time.time() >= timeout:
						print "Object not found - moving on..."
						break
			else:
				print "Received pose update"
				grasper.refine_pose()
				grasper.ur5.addMesh(grasper.showObjPose,"/dope_object_pose_carrier")
				if grasper.make_grasp() == True:
					##### Move the robot up and to transport-pose
					grasper.ur5.move_xyz_base_link_ur(0, 0, 0.05)
					#grasper.ur5.moveToDrivePose()

		elif inp == 'b':
			grasper.hasGraspedPub.publish(Bool(True))
			grasper.hasPutPub.publish(Bool(True))
			grasper.ur5.moveToSearchPose(putDownGoal.orientation)
			rospy.sleep(0.1)	# wait to arrive at position
			grasper.publish_image()
			timeout = time.time() + 1.5   # 1.5 seconds from now
			# Wait until updated pose arrives or timeout occurs (pose not visible)
			while grasper.poseIsUpdated_holder == False and time.time() <= timeout:
				rospy.sleep(0.05)
			posID = 0
			while grasper.poseIsUpdated_holder == False:		# if the object has been found, this is going to be True and outer loop is exited
				if grasper.ur5.searchObject(posID) == False:
					print "No object found!"
					break
				posID = posID + 1
				rospy.sleep(0.1) 	# wait to arrive at position
				grasper.publish_image()
				timeout = time.time() + 1.5   # 1.5 seconds from now
				# Wait until updated pose arrives or timeout occurs (pose not visible)
				while grasper.poseIsUpdated_holder == False:
					rospy.sleep(0.05)
					if time.time() >= timeout:
						print "Object not found - moving on..."
						break
				else:
					print "Received pose update"
					grasper.refine_pose()
					if grasper.unstore() == True:
						if grasper.put_down(putDownGoal.orientation) == True:
							##### Move the robot up and to transport-pose
							grasper.ur5.move_to_pose(grasper.postPutPose)
							#grasper.ur5.moveToDrivePose()

if __name__ == '__main__':
	main(sys.argv)


	'''pose = Pose()
	pose.position.x = 0.6
	pose.position.y = 0.1
	pose.position.z = 0.37
	pose.orientation.x = 0.#-0.4778
	pose.orientation.y = 0.707#0.4731
	pose.orientation.z = 0#-0.5263
	pose.orientation.w = 0.707#0.5203
	goalPose = [0.6, 0.1, 0.37, 0, 0, 0.707, 0.707]'''
