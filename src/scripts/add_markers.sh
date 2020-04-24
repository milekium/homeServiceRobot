#!/bin/sh

#xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore"  

xterm -e " source ./devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$ENV_SRC_FILES/map/world.world " &
sleep 5
xterm -e " source ./devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$ENV_SRC_FILES/map/map.yaml" &
sleep 5
xterm -e " source ./devel/setup.bash; rosrun rviz rviz -d $ENV_SRC_FILES/rvizConfig/config.rviz " &
sleep 5
xterm -e " source ./devel/setup.bash; roslaunch add_markers add_markers.launch" &

