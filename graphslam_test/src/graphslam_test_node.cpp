#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tfMessage.h>

ros::Publisher gmap_publisher;
ros::Publisher gmapmeta_publisher;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("Laser Scan message received: %s", msg->header.frame_id.c_str());
	nav_msgs::OccupancyGrid gmap_msg;
        //assign header
	gmap_msg.header = msg->header;

	//Create map
	gmap_msg.data.resize(20);
	for(int i = 0; i < 20; i++)
        {
                gmap_msg.data[i] = i;//Probability is in the range of [0,100]
        }

	//Create map meta data
	nav_msgs::MapMetaData gmapmeta_msg;

	gmapmeta_msg.map_load_time.sec = 0; //time at which the map was loaded
	gmapmeta_msg.map_load_time.nsec = 0; //time at which the map was loaded

	gmapmeta_msg.resolution = 0.05; //Cell resolution we think
	gmapmeta_msg.width = 5;
	gmapmeta_msg.height = 4;

	gmapmeta_msg.origin.position.x = -1;
	gmapmeta_msg.origin.position.y = -2;
	gmapmeta_msg.origin.position.z = 0;

	gmapmeta_msg.origin.orientation.x = 0;
	gmapmeta_msg.origin.orientation.y = 0;
	gmapmeta_msg.origin.orientation.z = 0;
	gmapmeta_msg.origin.orientation.w = 1;


	//assign map meta data
	gmap_msg.info = gmapmeta_msg;


	//publish
	gmap_publisher.publish(gmap_msg);
	gmapmeta_publisher.publish(gmapmeta_msg);
}

void tfCallback(const tf::tfMessage::ConstPtr& msg)
{
	//ROS_INFO("tf message received: %s", msg->transforms[0].header.frame_id.c_str());
	//ROS_INFO("tf x: %.3f", msg->transforms[0].transform.translation.x);
	//ROS_INFO("tf y: %.3f", msg->transforms[0].transform.translation.y);
	//ROS_INFO("tf z: %.3f", msg->transforms[0].transform.translation.z);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "graphslam_test_node");

	ros::NodeHandle node_handle;

	ROS_INFO("Started graphslam_test_node");

	gmap_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("grid_map", 1000);//True for latched
	gmapmeta_publisher = node_handle.advertise<nav_msgs::MapMetaData>("grid_mapmeta", 1000);//true for latched

	ros::Subscriber laserscan_subscriber = node_handle.subscribe<sensor_msgs::LaserScan>("scan", 1000, laserScanCallback);
	//TODO: Use transform listener instead!
	ros::Subscriber tf_subscriber = node_handle.subscribe<tf::tfMessage>("tf", 1000, tfCallback);

	ROS_INFO("Subscribers and Publishers initialized");

	while(ros::ok())
	{
		//Do stuff here.
		ros::spinOnce();
	}
	return 0;
}
