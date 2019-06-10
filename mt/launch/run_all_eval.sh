#!/bin/bash
xterm -hold -e "roscore" &
sleep 3

echo "Loading parameters"
xterm -hold -e "roslaunch mt load_config.launch" &

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

echo "Launching dope and tf-helper functions"
xterm -hold -e "roslaunch mt grasp_nodes.launch" &
xterm -hold -e "rosrun dope dope.py" &

#--------------------#
#--- We are done! ---#
#--------------------#
echo "----- Finished launching basic nodes! -----"
echo "Now run the following command"
echo "    rosrun mt grasp_object.py"
wait
