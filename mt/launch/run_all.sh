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

echo "Launching Camera and attaching it to robot..."
xterm -hold -e "roslaunch mt rs_aligned_depth.launch" &
xterm -hold -e "roslaunch mt tf_publishCam.launch" &
sleep 2

echo "Launching Marker-Detection for Pose-Estimation..."
xterm -hold -e "roslaunch mt aruco_detect.launch" &

echo "Launching tf-Marker-Broadcaster and tf-Listener"
# Refines marker-pose with depth-image and broadcasts pose
xterm -hold -e "rosrun mt tf_marker_broadcaster.py" &
# Publish base-object, object-cam, objectImgCenter-cam
xterm -hold -e "rosrun mt tf_listener_publisher.py" &

#--------------------#
#--- We are done! ---#
#--------------------#
echo "----- Finished launching basic nodes! -----"
echo "Now run the following commands in different consoles"
echo "    rosrun mt get_gtPose.py"
echo "    rosrun mt capture_pose_calc.py"
echo "    rosrun mt data_capture.py"
wait
