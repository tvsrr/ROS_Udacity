#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <unistd.h>  
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


double odom_robotx = 0.0, odom_roboty = 0.0;

void process_odom_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_robotx = msg->pose.pose.position.x;
  odom_roboty = msg->pose.pose.position.y;
}
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void reach_goal(move_base_msgs::MoveBaseGoal goal, string goal_name, MoveBaseClient &ac)
{
  ROS_INFO("Initiating goal reaching for location: %s", goal_name.c_str());
  ac.sendGoal(goal);
  

  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (goal_name == "pickup_location")
      ROS_INFO("Successfully reached Pickup Location");
    else 
      ROS_INFO("Successfully reached Drop Location");
  }
  else
  {
    if (goal_name == "pickup_location")
      ROS_INFO("Failed to reach Pickup Location");
    else 
      ROS_INFO("Failed to reach Drop Location");
  }
  ros::Duration(5.0).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(1);


  MoveBaseClient ac("move_base", true);


  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

 
  ros::Subscriber odom_pose_sub = n.subscribe("odom", 10, process_odom_pose_callback);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  while (ros::ok() && odom_robotx == 0.0 && odom_roboty == 0.0)
  {
    ros::spinOnce();
    r.sleep();
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";


  string goal_name = "pickup_location";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0262432694435;
  goal.target_pose.pose.position.y = -2.03702068329;

  geometry_msgs::PointStamped odom_point, map_point;
  odom_point.header.frame_id = "odom";
  odom_point.header.stamp = ros::Time(0);
  odom_point.point.x = odom_robotx;
  odom_point.point.y = odom_roboty;
  odom_point.point.z = 0.0;

  tfBuffer.transform(odom_point, map_point, "map", ros::Duration(0.2));

  double deltax = goal.target_pose.pose.position.x - map_point.point.x;
  double deltay = goal.target_pose.pose.position.y - map_point.point.y;
  double desired_yaw = atan2(deltay, deltax);
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(desired_yaw);

  ROS_INFO("Sending Pickup Location");
  reach_goal(goal, goal_name, ac);


  goal_name = "drop_location";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -2.6096;
  goal.target_pose.pose.position.y = -4.2139;

  //updated robot pose 
  odom_point.header.stamp = ros::Time(0);
  odom_point.point.x = odom_robotx;
  odom_point.point.y = odom_roboty;
  tfBuffer.transform(odom_point, map_point, "map", ros::Duration(0.2));
  deltax = goal.target_pose.pose.position.x - map_point.point.x;
  deltay = goal.target_pose.pose.position.y - map_point.point.y;
  desired_yaw = atan2(deltay, deltax);
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(desired_yaw);

  ROS_INFO("Sending Drop Location");
  reach_goal(goal, goal_name, ac);

  return 0;
}

