#include "ros/ros.h"
#include "std_srvs/Empty.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unpause_physics");
  ros::NodeHandle nh;

  ros::service::waitForService("/gazebo/unpause_physics");
  ros::ServiceClient unpauseClient = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

  std_srvs::Empty srv;
  if (unpauseClient.call(srv))
  {
    ROS_INFO("Successfully unpaused Gazebo physics.");
  }
  else
  {
    ROS_ERROR("Failed to call service /gazebo/unpause_physics.");
  }

  return 0;
}

