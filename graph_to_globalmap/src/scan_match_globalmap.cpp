#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>

#define PUBLISH_LOCAL_MAPS 0

//Map info
#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define MAP_RESOLUTION 0.05f

//Scan collection
#define NUM_GRAPH_POINTS 200
#define DIST_BETWEEN_POINTS 0.1f

//Macros
#define SQUARED(x) x*x

//Topics
#define LASERSCAN_TOPIC "scan"
#define ODOMETRY_TOPIC "odom"
#define GLOBALMAP_TOPIC "globalmap"

typedef struct {
	float dx;
	float dy;
	float yaw;
}Position;


//@Todo: This stores duplicate information data. We only need to store that information once. Then we should store only the actual sensor data for each node.
sensor_msgs::LaserScan* sensor_data = new sensor_msgs::LaserScan[NUM_GRAPH_POINTS];
Position* relative_positions = new Position[NUM_GRAPH_POINTS];

int scan_index = 0;

float getYaw(const geometry_msgs::Quaternion quat)
{
	tf::Quaternion quat_tf;
	tf::quaternionMsgToTF(quat, quat_tf);

	double roll, pitch, yaw;

	tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

	//ROS_INFO("Euler Angles: (%.3f, %.3f, %.3f)", roll, pitch, yaw);
	return (float)yaw;
}

void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	if(scan_index >= NUM_GRAPH_POINTS)return;


	static float prev_x = 0;
	static float prev_y = 0;

	geometry_msgs::Point p = odom_msg->pose.pose.position;

	//Calculate relative position
	float dx = prev_x - p.x;
	float dy = prev_y - p.y;

	//Using distance squared. Costly to use square root if we don't need to.
	float distance_squared = dx*dx + dy*dy;

	//@Note: This won't save the first position. It will start saving from a distance NUM_GRAPH_POINTS away from (0,0)
	if(distance_squared >= SQUARED(DIST_BETWEEN_POINTS))
	{
		ROS_INFO("%d | odometry (%f, %f) | relative position (%f, %f)", scan_index, p.x, p.y, dx, dy);

		//Save position to compute relative distance for next scan
		prev_x = p.x;
		prev_y = p.y;

		//Store relative position
		relative_positions[scan_index].dx = dx;
		relative_positions[scan_index].dy = dy;
		relative_positions[scan_index].yaw = getYaw(odom_msg->pose.pose.orientation);

		//Store laser scan
		sensor_data[scan_index] = *scan_msg;

		scan_index++;
	}
}

#if PUBLISH_LOCAL_MAPS
void rayTraceGlobalMap(ros::Publisher& gridmap_pub, ros::Publisher local_maps[NUM_GRAPH_POINTS])
#else
void rayTraceGlobalMap(ros::Publisher& gridmap_pub)
#endif
{
	//generate map
	nav_msgs::OccupancyGrid globalmap_msg;
	globalmap_msg.info.width = MAP_WIDTH;
	globalmap_msg.info.height = MAP_HEIGHT;
	globalmap_msg.info.resolution = MAP_RESOLUTION;

	//Resize the array to hold the map size. Set default value 50, for unknown
	globalmap_msg.data.resize(MAP_WIDTH * MAP_HEIGHT, 50);


	//@Todo:don't use this. --  Not optimízing, don't use this for final code.
	nav_msgs::OccupancyGrid num_for_average;
	num_for_average.data.resize(MAP_WIDTH * MAP_HEIGHT, 0);


	//Start at center of map
	int x0 = MAP_WIDTH / 2;
	int y0 = MAP_HEIGHT / 2;

	int x_start = x0;
	int y_start = y0;
	ROS_INFO("Generating globalmap from graph");

	//For debugging only use obstacle iformation within 10m radius.
	float sensor_range = 10.0f / MAP_RESOLUTION;


	//Loop through each scan
	for(int current_scan = 0; current_scan < NUM_GRAPH_POINTS; current_scan++)
	{
		#if PUBLISH_LOCAL_MAPS
		nav_msgs::OccupancyGrid local_map;
		local_map.info.width = MAP_WIDTH;
		local_map.info.height = MAP_HEIGHT;
		local_map.info.resolution = MAP_RESOLUTION;

		//Resize the array to hold the map size. Set default value 50, for unknown
		local_map.data.resize(MAP_WIDTH * MAP_HEIGHT, 50);
		#endif

		ROS_INFO("Ray tracing for sensor %d", current_scan);
		sensor_msgs::LaserScan& scan = sensor_data[current_scan];

		float angle_range = (scan.angle_max - scan.angle_min);
		int num_angles = (angle_range)/scan.angle_increment;


		Position relative_pos = relative_positions[current_scan];

		//Move from previous position to current position by adding the relative position.
		x_start += relative_pos.dx / MAP_RESOLUTION;
		y_start += relative_pos.dy / MAP_RESOLUTION;

		//Offset scan angles by robot's orientation
		float robot_angle = relative_pos.yaw + scan.angle_min;

		//Ray trace for current sensor
		for(int i = 0; i < num_angles; i++)
	        {
        	        float xDist = 0;
                	float yDist = 0;

	                int xa = x_start;
	                int ya = y_start;

			//Take into account the current rotation of the robot
	                float angle = robot_angle + i * scan.angle_increment;
			float dy = sin(angle);
                	float dx = cos(angle);

                	float dist_to_obstacle = scan.ranges[i]/MAP_RESOLUTION;

			//Go step by step along ray, but make sure to be in bounds of the gridmap
	                while(xa >= 0 && xa < MAP_WIDTH && ya >= 0 && ya < MAP_HEIGHT)
        	        {
				int index = xa + ya * globalmap_msg.info.width;
                        	float dist_travled = std::sqrt(xDist*xDist + yDist*yDist);

				if(dist_travled >= sensor_range)break;

				//Check if has reached the obstacle
                       		if(floor(dist_travled) >= floor(dist_to_obstacle))
                        	{
                                	if(num_for_average.data[index] == 0)
						//Overwrite if first time cell has been reached
						globalmap_msg.data[index] = 1;
					else
						globalmap_msg.data[index] += 1;
					num_for_average.data[index] += 1;

					#if PUBLISH_LOCAL_MAPS
					local_map.data[index] = 100;
					#endif

                                	//this ray has reached it's destination, go to next ray.
					break;
                        	}


				if(num_for_average.data[index] == 0)
					//Overwrite value with 0 only if it was the first time cell had been reached
					globalmap_msg.data[index] = 0;
				else
				num_for_average.data[index] += 1;

				#if PUBLISH_LOCAL_MAPS
				local_map.data[index] = 0;
				#endif


				//update travel distance
				xDist += dx;
                        	yDist += dy;

                        	xa = x_start + xDist;
                        	ya = y_start + yDist;
                	}

	        }
		#if PUBLISH_LOCAL_MAPS
		local_maps[k].publish(local_map);
		#endif
	}


	//@Todo: can be optimized a lot
	//Compute average of sensor information for each cell
	for(int i = 0; i < MAP_WIDTH*MAP_HEIGHT; i++)
	{
		//Don't need to compute average if cell hasn't been reached by a ray. It's unknown: 50
		if(num_for_average.data[i] == 0)continue;

		//Compute average for cell and rescale to range 0 -> 100
		float average = (float)globalmap_msg.data[i] / (float)num_for_average.data[i];
		average *= 100;

		//Store average
		globalmap_msg.data[i] = (char)average;
	}

	//Publish global map
	gridmap_pub.publish(globalmap_msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "graph_to_globalmap");

	ros::NodeHandle node_handle;

	message_filters::Subscriber<sensor_msgs::LaserScan> sensor_sub(node_handle, LASERSCAN_TOPIC, 1);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node_handle, ODOMETRY_TOPIC, 1);

	ros::Publisher globalmap_pub = node_handle.advertise<nav_msgs::OccupancyGrid>(GLOBALMAP_TOPIC, 1, true);

#if PUBLISH_LOCAL_MAPS
	ros::Publisher local_maps[NUM_GRAPH_POINTS];
	//local map publishers
	for(int i = 0; i < NUM_GRAPH_POINTS; i++)
	{
		std::ostringstream ss;
		ss << "localmap_";
		ss << i;
		std::string topic_name = ss.str();
		ROS_INFO("TOPIC: %s", topic_name.c_str());
		local_maps[i] = node_handle.advertise<nav_msgs::OccupancyGrid>(topic_name.c_str(), 1, true);
	}
#endif

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> mypolicy;
	message_filters::Synchronizer<mypolicy> sync(mypolicy(10), sensor_sub, odom_sub);

	sync.registerCallback(boost::bind(&scanOdomCallback, _1, _2));
	float sensor_range = 10.0f;
	while(ros::ok())
	{
		if(scan_index == NUM_GRAPH_POINTS)
		{
		#if PUBLISH_LOCAL_MAPS
			rayTraceGlobalMap(globalmap_pub, local_maps);
		#else
			rayTraceGlobalMap(globalmap_pub);
		#endif

			scan_index++;
		}
		ros::spinOnce();
	}

	delete[] sensor_data;
	delete[] relative_positions;

	return 0;
}
