#!/bin/bash
#xterm -hold -e "roscore" &
#sleep 3
echo "Connecting to UR5 and launching path-planner..."
xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
