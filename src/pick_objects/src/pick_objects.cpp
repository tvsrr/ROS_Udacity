#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;


double robotx = 0.0, roboty = 0.0;
bool amcl_pose_received = false;  

void process_amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  robotx = msg.pose.pose.position.x;
  roboty = msg.pose.pose.position.y;
  amcl_pose_received = true;
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

 
  ros::Subscriber amcl_pose_sub = n.subscribe("amcl_pose", 10, process_amcl_pose_callback);


  while (ros::ok() && !amcl_pose_received)
  {
    ros::spinOnce();
    r.sleep();
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";


  string goal_name = "pickup_location";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0262432694435;
  goal.target_pose.pose.position.y = -1.03702068329;


  double deltax = goal.target_pose.pose.position.x - robotx;
  double deltay = goal.target_pose.pose.position.y - roboty;
  double desired_yaw = atan2(deltay, deltax);
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(desired_yaw);

  ROS_INFO("Sending Pickup Location");
  reach_goal(goal, goal_name, ac);


  goal_name = "drop_location";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0291333198547;
  goal.target_pose.pose.position.y = 5.45930576324;
  deltax = goal.target_pose.pose.position.x - robotx;
  deltay = goal.target_pose.pose.position.y - roboty;
  desired_yaw = atan2(deltay, deltax);
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(desired_yaw);

  ROS_INFO("Sending Drop Location");
  reach_goal(goal, goal_name, ac);

  return 0;
}

