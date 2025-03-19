#!/bin/bash
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot small_world.launch'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot amcl.launch'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; rosrun add_markers add_markers_time_test'" 

