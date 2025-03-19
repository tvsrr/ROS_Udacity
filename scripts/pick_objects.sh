#!/bin/bash
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot small_world.launch'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; rosrun pick_objects pick_objects'" &
sleep 5
xterm -hold -e "bash -c 'source ~/ROS_Udacity/devel/setup.bash; roslaunch my_robot amcl.launch'" 

