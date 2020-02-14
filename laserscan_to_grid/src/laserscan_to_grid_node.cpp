#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

#define DEBUG 0


#if DEBUG
#define SCANGRID_TOPIC "map"
#else
#define SCANGRID_TOPIC "scan_gridmap"
#endif

#define LASERSCAN_TOPIC "scan"


ros::Publisher gridmap_publisher;

void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{


	int num_angles = (scan_msg->angle_max - scan_msg->angle_min)/scan_msg->angle_increment;

	/*
	ROS_INFO("Laserscan | min_range=%.3f | max_range=%.3f | angles_range=%.3f | size=%d",
	scan_msg->range_min, scan_msg->range_max, scan_msg->angle_max - scan_msg->angle_min, num_angles);
	*/


	//Create OccupancyGrid from laserscan data
	nav_msgs::OccupancyGrid scangrid_msg;

	//copy header
	scangrid_msg.header = scan_msg->header;
#if DEBUG
	scangrid_msg.header.frame_id = "map"; //debug test with map server only
#endif

	//Create MapMetaData assign width, height and cellsize
	scangrid_msg.info.width = 400; //width = 400 cells, cellsize=0.05m => 20m
	scangrid_msg.info.height = 400; //height = 400 cells, cellsize=0.05m => 20m
	scangrid_msg.info.resolution = 0.05f;

	//Fill grid with data from laserscans
	scangrid_msg.data.resize(scangrid_msg.info.width * scangrid_msg.info.height, 50); //fill with 50 = unknown
	int x0 = 400/2;
	int y0 = 400/2;
	float range_amount = (scan_msg->range_max - scan_msg->range_min);

#if DEBUG
	static int temp = 0;
#endif

	for(int i = 0; i < num_angles; i++)
	{
		float xDist = 0;
		float yDist = 0;
		int xa = x0;
		int ya = y0;
		float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
		float dy = sin(angle);
		float dx = cos(angle);
#if DEBUG
//		if(temp == 0)ROS_INFO("Angle: %.3f | dx: %.3f | dy: %.3f ", angle, dx, dy);
		float obstacle_dist = scan_msg->ranges[i]/0.05;//5/0.05;//scan_msg->ranges[i]/0.05;
#else
		float obstacle_dist = scan_msg->ranges[i]/0.05;
#endif
		int y_min = 400;
		int y_max = 0;
		int x_min = 400;
		int x_max = 0;
		while(xa >= 0 && xa < 400 && ya >= 0 && ya < 400)
		{
			if(xa > x_max)x_max = xa;
			if(ya > y_max)y_max = ya;
			if(xa < x_min)x_min = xa;
			if(ya < y_min)y_min = ya;
			int value = 0;
			float dist = std::sqrt(xDist*xDist + yDist*yDist);

			if(floor(dist) >= floor(obstacle_dist))
			{
				scangrid_msg.data[xa + ya * scangrid_msg.info.width] = 100;
			}
			scangrid_msg.data[xa + ya * scangrid_msg.info.width] = 0;
			xDist += dx;
			yDist += dy;

			xa = x0 + xDist;
			ya = y0 + yDist;
		}
#if DEBUG
		if(temp == 0)ROS_INFO("Angle: %.3f | dx: %.3f | dy: %.3f | xmin: %d | xmax: %d | ymin: %d | ymax : %d", angle, dx, dy, x_min, x_max, y_min, y_max);
#endif
	}


#if DEBUG
	temp++;
#endif

	//Publish generated grid
	gridmap_publisher.publish(scangrid_msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "laserscan_to_grid");

	ros::NodeHandle node_handle;

	gridmap_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>(SCANGRID_TOPIC, 1, false);

	ros::Subscriber laserscan_subscriber = node_handle.subscribe(LASERSCAN_TOPIC, 1, laserscan_callback);


	while(ros::ok())
	{
		ros::spinOnce();
	}


	return 0;
}
