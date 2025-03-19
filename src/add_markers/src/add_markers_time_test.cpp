#include <ros/ros.h>
#include <cmath>
#include <unistd.h>  // For sleep()

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

// TF2 headers
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// We'll store the raw odometry info in these variables.
double odomRobotX = 0.0, odomRobotY = 0.0;

// This callback just updates the raw odometry in the odom frame.
void process_odom_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odomRobotX = msg->pose.pose.position.x;
  odomRobotY = msg->pose.pose.position.y;
}

// Returns true if the distance between (a1,b1) and (a2,b2) is < 0.2.
bool location_reached(double a1, double b1, double a2, double b2)
{
  double dist = std::sqrt(std::pow((a1 - a2), 2) + std::pow((b1 - b2), 2));
  return dist < 0.2;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers_time_test");
  ros::NodeHandle n;
  ros::Rate r(1);

  // Publisher for the Marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscriber for the odometry in /odom frame
  ros::Subscriber odom_pose_sub = n.subscribe("odom", 10, process_odom_pose_callback);

  // TF2: create a buffer and a listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Marker definition
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map"; // Position is in map frame
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0262432694435;
  marker.pose.position.y = -1.03702068329;
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

  // Wait for a subscriber to Marker topic
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  // Publish the initial marker
  marker.header.stamp = ros::Time::now();
  marker_pub.publish(marker);

  bool item_picked = false;
  bool simulate_drop = false;

  ROS_INFO("Marker placed at (%.2f, %.2f) in map frame", marker.pose.position.x, marker.pose.position.y);

  while (ros::ok())
  {
    ros::spinOnce();

    // Create a point in the odom frame from the raw odom data
    geometry_msgs::PointStamped odom_point, map_point;
    odom_point.header.frame_id = "odom";
    odom_point.header.stamp = ros::Time(0); // or ros::Time::now() if needed
    odom_point.point.x = odomRobotX;
    odom_point.point.y = odomRobotY;
    odom_point.point.z = 0.0;

    // Attempt to transform that point into the map frame
    bool transform_success = true;
    try
    {
      tfBuffer.transform(odom_point, map_point, "map", ros::Duration(0.2));
      // now map_point is in map frame
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Could not transform odom -> map: %s", ex.what());
      transform_success = false;
    }

    if (transform_success)
    {
      // Compare the transformed robot position in map frame with marker pose in map frame
      double distance = std::sqrt(std::pow(map_point.point.x - marker.pose.position.x, 2) +
                                  std::pow(map_point.point.y - marker.pose.position.y, 2));
      bool status_flag = distance < 0.2;

      ROS_INFO("Robot(map): (%.2f, %.2f), Marker(map): (%.2f, %.2f), Dist=%.3f, Reached=%s",
               map_point.point.x, map_point.point.y,
               marker.pose.position.x, marker.pose.position.y,
               distance, status_flag ? "YES" : "NO");

      // Pickup phase
      if (!item_picked)
      {
        if (status_flag)
        {
          // "Delete" the marker to simulate pickup
          marker.action = visualization_msgs::Marker::DELETE;
          marker.header.stamp = ros::Time::now();
          marker_pub.publish(marker);
          ROS_INFO("pick_up successful!");
          
          // Wait for 5 seconds to simulate travel time
          ros::Duration(5.0).sleep();

          item_picked = true;

          // Set marker position for drop off
          marker.pose.position.x = 4.6096;
          marker.pose.position.y = -3.139;

          ROS_INFO("Assuming vehicle will reach drop location in 5 seconds...");
          ros::Duration(5.0).sleep();

          simulate_drop = true; // We'll rely on "simulate_drop" to do the drop
        }
      }
      else
      {
        // Drop-off phase
        bool drop_status = false;
        if (simulate_drop)
        {
          // If you want to also wait for the location to be reached again, do:
          // drop_status = status_flag;
          // Or just skip it and rely on "simulate_drop" alone.
          drop_status = true;
        }
        if (drop_status)
        {
          marker.action = visualization_msgs::Marker::ADD;
          marker.header.stamp = ros::Time::now();
          marker_pub.publish(marker);
          ROS_INFO("Drop-off zone reached: marker published (dropped off).");
          ros::Duration(5.0).sleep();
          break; // end the loop
        }
      }
    }

    r.sleep();
  }

  return 0;
}

