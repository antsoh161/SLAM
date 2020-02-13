//@NOTE: Must be faster than 10-15hz
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tfMessage.h>

//define topic names
#define POSE_TOPIC "pose"
#define PARTICLECLOUD_TOPIC "particlecloud"
#define LASERSCAN_TOPIC "scan"
#define TF_TOPIC "tf"
#define MAP_TOPIC "map"
//@TODO: check what this topic is called
#define INITIALPOSE_TOPIC "initialpose"

//subscriber topic callbacks
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{ 
        ROS_INFO("MAP_CALLBACK");
}

void tf_callback(const tf::tfMessage::ConstPtr& tf_msg)
{ 
        //ROS_INFO("TF_CALLBACK");
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	//ROS_INFO("SCAN_CALLBACK");
}

void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initialpose_msg)
{ 
        ROS_INFO("INITIALPOSE_CALLBACK");
}


//entry point
int main(int argc, char* argv[])
{
	//Initializa ros node
	ros::init(argc, argv, "particlefilter_test_node");

	ros::NodeHandle node_handle;

	ROS_INFO("Started particlefilter_test_node");


	//Setup publishers and subscribers
	ros::Publisher pose_publisher = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(POSE_TOPIC, 1);

	ros::Publisher particlecloud_publisher = node_handle.advertise<geometry_msgs::PoseArray>(PARTICLECLOUD_TOPIC, 1);

	ros::Publisher tf_publisher = node_handle.advertise<tf::tfMessage>(TF_TOPIC, 1);

	ros::Subscriber laserscan_subscriber = node_handle.subscribe(LASERSCAN_TOPIC, 1, scan_callback);

	ros::Subscriber tf_subscriber = node_handle.subscribe(TF_TOPIC, 1, tf_callback);

	ros::Subscriber initialpose_subscriber = node_handle.subscribe(INITIALPOSE_TOPIC, 1, initialpose_callback);

	ros::Subscriber map_subscriber = node_handle.subscribe(MAP_TOPIC, 1, map_callback);


	//Ros loop
	while(ros::ok())
	{
		geometry_msgs::PoseWithCovarianceStamped new_pose_msg;
		pose_publisher.publish(new_pose_msg);

		geometry_msgs::PoseArray new_pcloud_msg;
		particlecloud_publisher.publish(new_pcloud_msg);

		tf::tfMessage new_tf_msg;
		tf_publisher.publish(new_tf_msg);
		ros::spinOnce();
	}

	return 0;
}
