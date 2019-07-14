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

# IF IMPORT NOT WORKING:
sys.path.append('/home/johannes/git/mt/mt/src/')
from mir_control import mir_control
from mission_control import mission_control
from grasp_control import grasp_control



# Init node
rospy.init_node('transport_object', anonymous=True, disable_signals=True)
rospy.loginfo("Make sure correct camera intrinsics are set in yaml file!")

mir = mir_control.mirControler()
grasper = grasp_control.grasp_process()



def drive_and_search(goal, obj):
	### Move MiR
	print "Sending MiR to goal..."
	mir.moveMiR(goal)

	### Search for object
	print "Searching for object..."
	if grasper.search(goal, obj) == False:
		return False

	### Refining pose
	print "Refining pose..."
	grasper.refine_pose(obj)

	return True

def transport(pickUpGoal, putDownGoal):
	### Reset grasper state
	grasper.hasGraspedPub.publish(Bool(True))
	grasper.hasPutPub.publish(Bool(True))
	grasper.ur5.removeAttachedObject()
	grasper.ur5.scene.remove_world_object()

	# Search for carrier and add it to scene
	if drive_and_search(pickUpGoal, "carrier") == False:
		return False
	grasper.ur5.addMesh(grasper.showObjPose,"/dope_object_pose_carrier")

	### Grasping object
	if grasper.make_grasp() == False:
		return False
		
	### Storing object and moving to driving pose
	print "Storing object..."
	grasper.store()
	print "Moving to driving pose..."
	grasper.ur5.moveToDrivePose()

	# Search for holder
	if drive_and_search(putDownGoal, "holder") == False:
		return False

	# Pick up object
	if grasper.unstore() == False:
		return False
	
	# Put it down
	print "Putting down object..."
	grasper.put_down(putDownGoal.orientation)

	# Move to driving pose
	print "Moving to driving pose..."
	grasper.ur5.moveToDrivePose()

	return True

def main(args):
	parser = argparse.ArgumentParser(description='Transport Object')
	parser.add_argument('pickUpGoal', metavar='pickUpGoal', type=str, help='Name of goal to pick up carrier')
	parser.add_argument('putDownGoal', metavar='putDownGoal', type=str, help='Name of goal to put down carrier')
	pickUpGoal = mission_control.goal(parser.parse_args().pickUpGoal)
	putDownGoal = mission_control.goal(parser.parse_args().putDownGoal)

	while (True):
		grasper.reset_poses()
		inp = raw_input("p to publish image, a for search+grasp, b for search+put, \
						 s to store, u to unstore, \
						 d to drive to pickUpGoal, e to drive to putDownGoal, \
						 c to make complete transport: ")[0]
		
		if inp == 'c':
			if transport(pickUpGoal, putDownGoal) == True:
				print "Success"

		if inp == 'd':
			mir.moveMiR(pickUpGoal)

		if inp == 'e':
			mir.moveMiR(putDownGoal)

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
			
			obj = "carrier"
			### Search for object
			print "Searching for object..."
			if grasper.search(goal, obj) == False:
				return False

			### Refining pose
			print "Refining pose..."
			grasper.refine_pose(obj)

			grasper.ur5.addMesh(grasper.showObjPose,"/dope_object_pose_carrier")

			### Grasping object
			if grasper.make_grasp() == False:
				return False

		elif inp == 'b':
			grasper.hasGraspedPub.publish(Bool(True))
			grasper.hasPutPub.publish(Bool(True))

			obj = "holder"
			### Search for object
			print "Searching for object..."
			if grasper.search(goal, obj) == False:
				return False

			### Refining pose
			print "Refining pose..."
			grasper.refine_pose(obj)

			# Pick up object
			if grasper.unstore() == False:
				return False
			
			# Put it down
			print "Putting down object..."
			grasper.put_down(putDownGoal.orientation)

			# Move to driving pose
			print "Moving to driving pose..."
			grasper.ur5.moveToDrivePose()


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
