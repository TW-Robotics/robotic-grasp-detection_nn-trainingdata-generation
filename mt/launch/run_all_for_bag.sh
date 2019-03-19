#!/bin/bash
xterm -hold -e "roscore" &
sleep 3
echo "Connecting to UR5 and launching path-planner..."
xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
sleep 3

# camera
xterm -hold -e "roslaunch mt rs_aligned_depth.launch" &
#xterm -hold -e "roslaunch mt img_rotate.launch" &
# Camera at eef
xterm -hold -e "roslaunch mt tf_publishCam.launch" &
# marker detection
xterm -hold -e "roslaunch aruco_detect aruco_detect.launch" &
# marker tf
xterm -hold -e "rosrun mt tf_marker_broadcaster.py" &
# Pose von erkannten Markern während GT-Bestimmung
xterm -hold -e "rosrun mt tf_gtPoseVGL_broadcaster.py" &
# Object-to-base_link and Object-to-cam topic-publisher
xterm -hold -e "rosrun mt tf_listener_publisher.py" &
# Caputre-Poses tf broadcaster
xterm -hold -e "rosrun mt tf_capturePose_broadcaster.py" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Finished launching basic nodes"
echo "Now run the following commands in different consoles"
echo "    rosrun mt get_gtPose.py"
echo "    rosrun mt capture_pose_calc.py"
echo "    rosrun mt data_capture.py"
wait
