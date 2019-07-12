#!/usr/bin/env python
import time
import roslib
import rospy
import sys
import actionlib
import copy
import tf

import moveit_commander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String
import math
from math import pi

# Sources:
# Moveit-MoveGroup-Commander: http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
# Moveit-Planning-Interface: http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html
# Planning Scene: http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
# To make mesh-importer working: https://launchpadlibrarian.net/319496602/patchPyassim.txt
#		open /usr/lib/python2.7/dist-packages/pyassimp/core.py and change line 33 according to the link to "load, release, dll = helper.search_library()"

''' Control the UR5 robot - basic functions (e.g. moveXYZ) '''

# Set debug = True to display debug-messages
debug = False

class ur5Controler():
	def __init__(self, eef_link="gripper", pose_ref_frame="/base_link", checkBeforeDo=True):
		# Set Robot paramters: position over floor, speed and acceloration [0, 1] (only 0.1 steps)
		self.speed = 0.1
		self.acceleration = 0.1
		self.speedScalingFactor = 0.05		# For timing of path-planning-points [very small eg 0.01, 1]

		#self.pathToObjectFile = "/home/johannes/catkin_ws/src/mt/cad/product.stl"
		objName = rospy.get_param("object_name")
		self.pathToObjectFile = rospy.get_param("path_to_obj_stl/" + str(objName))#"/home/mluser/catkin_ws/src/mt/cad/product.stl"

		# Set True to make the program ask before the robot moves
		self.checkBeforeDo = checkBeforeDo

		# Init moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		group_name = "manipulator"
		self.group = moveit_commander.MoveGroupCommander(group_name)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group.set_end_effector_link(eef_link)
		self.group.set_pose_reference_frame(pose_ref_frame)
		self.setSpeed(self.speed, self.acceleration)

		#print self.robot.get_planning_frame()
		#print self.group.get_pose_reference_frame()
		#print self.group.get_current_pose().pose
		#print self.group.get_end_effector_link()

	######################################################
	###### GENERAL FUNCTIONS FOR BASIC MOVING TASKS ######
	######################################################

	# Move robot to upright position
	def go_home(self):
		goalPose = [0, -1.565, 0, -1.569, 0, 0]
		self.execute_move(goalPose)

	# Move the robot to a safe position to drive around
	def moveToDrivingPose(self):
		jointStates = [0, -180*pi/180, 150*pi/180, -150*pi/180, -90*pi/180, 0]
		self.execute_move(jointStates)

	# Move robot to a specific pose
	def move_to_pose(self, goalPose):
		#print self.group.get_current_pose().pose

		if type(goalPose) is not Pose:
			goal_pose = Pose()
			goal_pose.position.x = goalPose[0]
			goal_pose.position.y = goalPose[1]
			goal_pose.position.z = goalPose[2]
			goal_pose.orientation.x = goalPose[3]
			goal_pose.orientation.y = goalPose[4]
			goal_pose.orientation.z = goalPose[5]
			goal_pose.orientation.w = goalPose[6]
		else:
			goal_pose = goalPose

		#print goal_pose
		self.execute_move(goal_pose)

	# Move to x/y/z-position (incremental)
	def move_xyz(self, x_inc, y_inc, z_inc):
		goal_pose = self.group.get_current_pose().pose

		goal_pose.position.x = goal_pose.position.x + x_inc
		goal_pose.position.y = goal_pose.position.y + y_inc
		goal_pose.position.z = goal_pose.position.z + z_inc

		self.execute_move(goal_pose)

	# Move along a cartesian path
	def move_cartesian_path(self, x_inc, y_inc, z_inc):
		# Init waypoints and scale
		waypoints = []
		scale = 1
		wpose = self.group.get_current_pose().pose

		wpose.position.x += scale * x_inc
		wpose.position.y += scale * y_inc
		wpose.position.z += scale * z_inc
		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = self.group.compute_cartesian_path(
										   waypoints,   # waypoints to follow
										   0.01,        # eef_step (0.01 means interpolate at a resolution of 1 cm)
										   0.0)         # jump_threshold (0 means disabled)

		# Execute the planned path
		self.execute_plan(plan)

	# Move one specific joint by a specific angle
	def move_joint(self, jointNr, angleDeg_inc):
		# Calculate the angle in radiant
		angleRad_inc = angleDeg_inc * pi / 180

		# Set goal to current joint values and overwrite the relevant angle
		goal_jointStates = self.group.get_current_joint_values()
		goal_jointStates[jointNr] = goal_jointStates[jointNr] + angleRad_inc

		# Call function to move robot
		self.execute_move(goal_jointStates)

	# Move one specific joint to a given goal angle
	def move_joint_to_target(self, jointNr, angleRad):
		# Set goal to current joint values and overwrite the relevant angle
		goal_jointStates = self.group.get_current_joint_values()
		goal_jointStates[jointNr] = angleRad

		# Call function to move robot
		self.execute_move(goal_jointStates)

	# Move the robot to goal pose or orientation
	def execute_move(self, goal):
		#self.setSpeed()

		# if the goal is a pose
		if type(goal) is Pose:
			self.group.set_pose_target(goal)
		else:
			self.group.set_joint_value_target(goal)
		plan = self.group.plan()	# Show move in rviz

		#rospy.sleep(0.05)	# Give time for keyboard-interrupt
		if self.confirmation(goal):
			self.group.go(wait=True)
			self.group.clear_pose_targets()
			self.group.stop()

	# Move the robot along a specified way (plan)
	def execute_plan(self, plan):
		# Retime all timestamps of the way-points to make the robot move at a specified speed
		plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.speedScalingFactor)

		#rospy.sleep(0.05)	# Give time for keyboard-interrupt
		if self.confirmation(plan):
			self.group.execute(plan, wait=True)

	# Check, if input necessary, if there is positive input
	def confirmation(self, goal):
		inp = ""
		if self.checkBeforeDo == True:
			if debug == True:
				if type(goal) != RobotTrajectory:
					print " *************************************** Current ***"
					if type(goal) is Pose:
						print self.group.get_current_pose().pose
					else:
						print self.group.get_current_joint_values()
					print " *************************************** Goal ***"
					print goal
			inp = raw_input("Move robot? y/n: ")[0]
		if (inp == 'y' or self.checkBeforeDo == False):
			print "Moving robot..."
			return True
		print "Aborted by user."
		return False

	# Define Speed and acceleration
	def setSpeed(self, speed, acceleration):
		self.group.set_max_velocity_scaling_factor(speed)
		self.group.set_max_acceleration_scaling_factor(acceleration)

	# Function to make box at specific position - currently not needed
	def addObject(self):
		rospy.sleep(2)
		obj_pose = PoseStamped()
		obj_pose.header.frame_id = self.robot.get_planning_frame()
		obj_pose.pose.orientation.w = 1.0
		obj_pose.pose.position.x = -0.2
		obj_pose.pose.position.y = 0
		obj_pose.pose.position.z = -0.3
		box_name = "box"
		self.scene.add_box(box_name, obj_pose, size=(0.6, 0.4, 0.6))
		rospy.sleep(1)
		
	def addMesh(self, pose, frame_id):
		rospy.sleep(2)
		obj_pose = PoseStamped()
		obj_pose.header.frame_id = frame_id
		obj_pose.pose.position.x = pose.position.x
		obj_pose.pose.position.y = pose.position.y
		obj_pose.pose.position.z = pose.position.z
		obj_pose.pose.orientation.x = pose.orientation.x
		obj_pose.pose.orientation.y = pose.orientation.y
		obj_pose.pose.orientation.z = pose.orientation.z
		obj_pose.pose.orientation.w = pose.orientation.w

		# Import the STL-File
		self.scene.add_mesh("object", obj_pose, self.pathToObjectFile, size=(0.001, 0.001, 0.001))

		rospy.sleep(1)
		#print self.scene.get_known_object_names()

	# Check if a given goal-pose is reachable
	def isReachable(self, goalPose):
		# Change planning-time to make process faster
		oldTime = self.group.get_planning_time()
		self.group.set_planning_time(0.1)
		self.group.set_pose_target(goalPose)
		plan = self.group.plan()
		self.group.clear_pose_targets()
		# Reset planning-time
		self.group.set_planning_time(oldTime)
		# Check if a valid trajectory has been found
		if len(plan.joint_trajectory.joint_names) == 0:
			return False
		return True


	#####################################################
	####### SPECIFIC FUNCTIONS FOR TASK EXECUTION #######
	#####################################################

	# Move the robot to the position, where it starts to search for the object
	def moveToSearchPose(self, orientation):
		# Drive to position where r = 0.4 and h = 0.6
		jointStates = [110*pi/180, -100*pi/180, 75*pi/180, -100*pi/180, -90*pi/180, 90*pi/180]

		if orientation == "left":
			jointStates[0] = 65*pi/180
		elif orientation == "right":
			jointStates[0] = -115*pi/180
		elif orientation == "front":
			jointStates[0] = -25*pi/180

		self.execute_move(jointStates)


	def moveToTransportPose(self):
		jointStates = [0*pi/180, -180*pi/180, 115*pi/180, -115*pi/180, -90*pi/180, 0*pi/180]
		self.execute_move(jointStates)		

	# Turn the robot around R1 and R4 to search the object
	def searchObject(self, num):
		if num == 0:
			return True
		elif num == 1:
			self.move_joint(0, 25)
		elif num == 2:
			self.move_joint(0, 25)
		elif num == 3:
			self.move_joint(3, 15)
		elif num == 4:
			self.move_joint(0, -25)
		elif num == 5:
			self.move_joint(0, -25)
		else:
			return False
		return True

	def attachObjectToEEF(self):
		eef_link = self.group.get_end_effector_link()
		self.scene.attach_mesh(eef_link, "object")

	def removeAttachedObject(self):
		eef_link = self.group.get_end_effector_link()
		self.scene.remove_attached_object(eef_link, "object")
	
def main(args):
	try:
		# Initialize ros-node and Class
		rospy.init_node('ur5Controler', anonymous=True, disable_signals=True)
		ur5 = ur5Controler()

		''' # Put Objects into the robots world
		ur5.scene.remove_world_object()
		ur5.attachEEF()
		ur5.addObject()
		
		# Move to up-Position
		print "Moving home"
		ur5.go_home()
		
		# Move to pose
		# Info: Get actual pose: rosrun tf tf_echo base_link tool0'''
		print "Moving to pose"
		goalPose = [0.6, 0.1, 0.37, 0, 0, 0.707, 0.707] # Point x, y, z in Meter; Orientation x, y, z, w in Quaternionen
		ur5.move_to_pose(goalPose)
		
		# Move to x/y/z-position (incremental)
		print "Moving xyz-incremental"
		x_inc = 0
		y_inc = 0.1
		z_inc = -0.1
		ur5.move_xyz(x_inc, y_inc, z_inc)

		# Move along a cartesian path
		'''print "Moving along cartesian path"
		ur5.move_cartesian_path(x_inc, y_inc, z_inc)
		
		# Move to joint-orientations
		print "Moving to joint-orientations"
		jointStates = [-1.13, -1.56, -0.65, -0.94, 1.81, 0.54] # R1-R6
		ur5.execute_move(jointStates)

		# Move one specific joint one specific angle
		print "Moving one joint"
		jointNr = 0			# 0 to 5
		angleDeg_inc = 90
		ur5.move_joint(jointNr, angleDeg_inc)
		
		ur5.go_home()'''
		

	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

if __name__ == '__main__':
	main(sys.argv)
