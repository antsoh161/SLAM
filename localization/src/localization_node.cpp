#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#define LASERSCAN_TOPIC "scan"
#define ODOM_TOPIC "odom"


void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  tf::TransformListener odom_listener;
  tf::StampedTransform transform;
  try{
        odom_listener.lookupTransform("/odom", "/base_link",
                                ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  // ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> y: [%f]", transform.getOrigin().y());

  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ROS_INFO("SFHAKHs\n");


	ros::Subscriber laserscan_subscriber = nh.subscribe(LASERSCAN_TOPIC, 1, laserscan_callback);
  while (ros::ok())
  {
    ros::spinOnce();
  }


  return 0;
}