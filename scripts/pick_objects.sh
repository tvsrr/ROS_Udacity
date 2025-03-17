#!/bin/bash
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot world.launch'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; rosrun pick_objects pick_objects'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot amcl.launch'" 

