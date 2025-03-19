#include <ros/ros.h>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <unistd.h>  
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// transformation error has to be fixed odom->map


using namespace std;

double odom_robotx = 0.0, odom_roboty = 0.0;

void process_odom_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_robotx = msg->pose.pose.position.x;
  odom_roboty = msg->pose.pose.position.y;
}


bool location_reached(double &a1, double &b1, double &a2, double &b2)
{
  return std::sqrt(std::pow((a1 - a2), 2) + std::pow((b1 - b2), 2)) < 0.2;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers_time_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_pose_sub = n.subscribe("odom", 10, process_odom_pose_callback);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  uint32_t shape = visualization_msgs::Marker::CUBE;

  
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0262432694435;
    marker.pose.position.y = -2.03702068329;
    marker.pose.position.z = 0.08;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    bool item_picked = false;
    bool status_flag = false;
    bool simulate_drop = false; 
	
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
	ROS_INFO("Marker placed at (%.2f, %.2f) in map frame", marker.pose.position.x, marker.pose.position.y);
	
	
    while (ros::ok())
    {
      // Update marker timestamp each iteration
      marker.header.stamp = ros::Time::now();
      ros::spinOnce();
	  geometry_msgs::PointStamped odom_point, map_point;
	  // creating odom point to facilitate tf
	  odom_point.header.frame_id = "odom";
      odom_point.header.stamp = ros::Time(0); 
      odom_point.point.x = odom_robotx;
      odom_point.point.y = odom_roboty;
      odom_point.point.z = 0.0;
	  tfBuffer.transform(odom_point, map_point, "map", ros::Duration(0.2));
	  
      status_flag = location_reached(map_point.point.x, map_point.point.y, marker.pose.position.x, marker.pose.position.y);
      if(status_flag){
        ROS_INFO("Location Reached Successfully!");
}


      // Pickup phase
      if (!item_picked)
      {
        if (status_flag)
        {
          marker.action = visualization_msgs::Marker::DELETE;
          marker_pub.publish(marker);

          ROS_INFO("pick_up successful!");
         
          ros::Duration(5.0).sleep();
          item_picked = true;
          // Set marker position for drop off
          marker.pose.position.x = -2.6096;
          marker.pose.position.y = -4.2139;
          ROS_INFO("Assuming Vehicle reached drop location in 5 seconds");
          ros::Duration(5.0).sleep(); 
          simulate_drop = true;
        }
      }
      // Drop-off phase
      else
      {
         if (status_flag || simulate_drop){
          marker.action = visualization_msgs::Marker::ADD;
          marker_pub.publish(marker);
          ROS_INFO("Drop-off zone reached: marker published (dropped off).");
          ros::Duration(5.0).sleep();
        break;
      }
      }
      ros::spinOnce();
      r.sleep();
    }
    
  return 0;
}
