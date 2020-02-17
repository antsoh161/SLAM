#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>

sensor_msgs::LaserScan scanTempArray[10];
float x_pos[10];
float y_pos[10];

int current_index = 0;

void syncedCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	if(current_index >= 10)return;
//	ROS_INFO("We are synchronized %s, %s", scan_msg->header.frame_id.c_str(), odom_msg->header.frame_id.c_str());
	geometry_msgs::Point p = odom_msg->pose.pose.position; 
	if(current_index == 0)
	{
	ROS_INFO("%f, %f, %f", p.x, p.y, p.z);

		scanTempArray[0] = *scan_msg;
		x_pos[0] = p.x;
		y_pos[0] = p.y;
		current_index++;
	}else if(current_index < 10)
	{
		if(std::abs(p.x - x_pos[current_index - 1]) >= 1 || std::abs(p.y - y_pos[current_index - 1]) >= 1 )
		{
	ROS_INFO("%f, %f, %f", p.x, p.y, p.z);
			scanTempArray[current_index] = *scan_msg;
			x_pos[current_index] = p.x;
			y_pos[current_index] = p.y;
			current_index++;
		}
	}

}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "graph_to_globalmap");

	ros::NodeHandle node_handle;

	//ros::Subscriber sensor_sub = node_handle.subscribe("scan", 1, sensor_callback);
	//ros::Subscriber odom_sub = node_handle.subscribe("odom", 1, odom_callback);

	message_filters::Subscriber<sensor_msgs::LaserScan> sensor_sub(node_handle, "scan", 1);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node_handle, "odom", 1);

	ros::Publisher globalmap_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("globalmap", 1, true);


	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> mypolicy;
	message_filters::Synchronizer<mypolicy> sync(mypolicy(10), sensor_sub, odom_sub);

	sync.registerCallback(boost::bind(&syncedCallback, _1, _2));
	float sensor_range = 10.0f;
	while(ros::ok())
	{
		if(current_index == 10)
		{
			nav_msgs::OccupancyGrid globalmap_msg;
			globalmap_msg.info.width = 1000;
			globalmap_msg.info.height = 1000;
			globalmap_msg.info.resolution = 0.05f;


			globalmap_msg.data.resize(1000 * 1000);

			for(int y = 0; y < 1000; y++)
			{
				float ya = (float)y / 0.05f; //to world coordinate
				for(int x = 0; x < 1000; x++)
				{
					float xa = (float)x / 0.05f; //to world coordinate
					int index = x + y * 1000;
					float cell_value = 0;
					int num_reaching_cell = 0;
					for(int i = 0; i < 10; i++)
					{
						float dx = xa - x_pos[i];
						float dy = ya - y_pos[i];
						float distance_from_sensor_reading = std::sqrt(dx*dx + dy*dy);
						if(distance_from_sensor_reading <= sensor_range)
						{
							//calculate angle from sensor reading to grid cell
							float angle = std::atan2(dy, dx);

							//get integer position from angle.
							int angle_index_0 = std::floor(angle / scanTempArray[i].angle_increment);
							int angle_index_1 = std::ceil(angle / scanTempArray[i].angle_increment);

							float distance_to_obstacle = scanTempArray[i].ranges[angle_index_0] + scanTempArray[i].ranges[angle_index_1];
							distance_to_obstacle /= 2.0f;
							float sensor_value = 0; //free
							if(floor(distance_from_sensor_reading) == floor(distance_to_obstacle))
							{
								sensor_value = 100;
							}
							else if(floor(distance_from_sensor_reading) > floor(distance_to_obstacle))
							{
								continue; //Unkown behind obstacle
							}


							cell_value += sensor_value;
							num_reaching_cell++;
						}

					}
					if(num_reaching_cell > 0)
					{
						cell_value /= num_reaching_cell;
					}else
					{
						cell_value = 50; //unkown otherwise
					}
					globalmap_msg.data[index] = cell_value;
				}
			}

			current_index++;
			globalmap_pub.publish(globalmap_msg);
		}
		ros::spinOnce();
	}
	return 0;
}
