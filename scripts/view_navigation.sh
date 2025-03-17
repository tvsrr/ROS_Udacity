#!/bin/bash
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot world.launch'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; rosrun add_markers add_markers'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot amcl.launch'" 
