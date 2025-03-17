#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void reach_goal(move_base_msgs::MoveBaseGoal goal, string goal_name, MoveBaseClient &ac){

ROS_INFO("Initiating Goal reaching for location: %s", goal_name.c_str());

ac.sendGoal(goal);
  
// Wait an infinite time for the results
ac.waitForResult();

// Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    if (goal_name == "pickup_location")
    ROS_INFO("Successfully reached Pickup Location");
    else 
    ROS_INFO("Successfully reached Drop Location");}
  else{
    if (goal_name == "pickup_location")
    ROS_INFO("Failed to reach Pickup Location");
    else 
    ROS_INFO("Failed to reach Drop Location");
}
    ros::Duration(5.0).sleep();

}



int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  string goal_name = "pickup_location";
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Pickup Location");
  reach_goal(goal, goal_name, ac);
  goal_name = "drop_location";
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 6.0;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending Drop Location");
  reach_goal(goal, goal_name, ac);

  return 0;
}
