#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

using namespace std;
// %EndTag(INCLUDES)%

double robotx = 0.0, roboty = 0.0;

void process_odom_data_callback(const nav_msgs::Odometry &msg)
{
  robotx = msg.pose.pose.position.x;
  roboty = msg.pose.pose.position.y;
}

bool location_reached(double &a1, double &b1, double &a2, double &b2){
 return std::sqrt(std::pow((a1 - a2), 2) + std::pow((b1 - b2), 2)) < 0.1;
}

// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber marker_sub = n.subscribe("odom", 10, process_odom_data_callback);

// %EndTag(INIT)%

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = visualization_msgs::Marker::CUBE;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
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
   while (ros::ok())
  {
    // Update marker timestamp each iteration
    marker.header.stamp = ros::Time::now();

    // Process any pending callbacks (odom)
    ros::spinOnce();

    // Check if the robot is at the marker's position
    bool status_flag = location_reached(robotx, roboty,
                                        marker.pose.position.x,
                                        marker.pose.position.y);

    // Pickup phase
    if (!item_picked)
    {
      // If robot is at pickup
      if (status_flag)
      {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        ROS_INFO("pick_up successful!");
        item_picked = true;
        ros::Duration(5.0).sleep();

        // Set marker position for drop off
        marker.pose.position.x = 6.0;
        marker.pose.position.y = 2.0;
      }
      else
      {
        // Continue displaying marker at pickup location
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
      }
    }
    // Drop-off phase
    else
    {
      // If robot is at drop-off
      if (status_flag)
      {
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);

        ROS_INFO("drop successful!");
        ros::Duration(5.0).sleep();
        break;  // End the program after dropping off
      }
      else
      {
        // Hide marker until robot reaches drop-off location
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
      }
    }
// %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}}
// %EndTag(FULLTEXT)%
