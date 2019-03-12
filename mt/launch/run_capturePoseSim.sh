#!/bin/bash
xterm -hold -e "roscore" &
sleep 3

echo "Connecting to UR5 and launching path-planner..."
xterm -hold -e "roslaunch ur_gazebo ur5.launch limited:=true" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
sleep 3

# Subscribe to calculated capture poses and broadcast them to tf
# Also broadcast object position
xterm -hold -e "rosrun mt tf_capturePose_broadcaster.py"  &
# Calculate transformation from base_link to the object and publish it as topic for capture_pose_calc_structured
xterm -hold -e "rosrun mt tf_listener_publisher.py" &
# Calculate capture poses and publish them
sleep 2
xterm -hold -e "rosrun mt capture_pose_calc_structured.py" &

#xterm -hold -e "roslaunch mt tf_transform.launch" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait



