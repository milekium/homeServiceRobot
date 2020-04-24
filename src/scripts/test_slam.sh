#!/bin/sh

#xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore"  

xterm -e " source ./devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$ENV_SRC_FILES/map/world.world " &
sleep 5
xterm -e " source ./devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e " source ./devel/setup.bash; rosrun rviz rviz -d $ENV_SRC_FILES/rvizConfig/config.rviz " &
sleep 5
xterm -e " source ./devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" 
