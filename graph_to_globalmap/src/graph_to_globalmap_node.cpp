#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
//#include <LinearMath/btMatrix3x3.h>

#define USE_FIRST 0
#define USE_RAY_TRACE 1
#define USE_RAY_TRACE_LOG_ODDS 0
#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define MAP_RESOLUTION 0.05f
#define NUM_GRAPH_POINTS 10
#define DIST_BETWEEN_POINTS 0.2f

sensor_msgs::LaserScan scanTempArray[NUM_GRAPH_POINTS];
float x_pos[NUM_GRAPH_POINTS];
float y_pos[NUM_GRAPH_POINTS];
float yaw[NUM_GRAPH_POINTS];

int current_index = 0;

float getYaw(const geometry_msgs::Quaternion quat)
{
	tf::Quaternion quat_tf;
	tf::quaternionMsgToTF(quat, quat_tf);

	double roll, pitch, yaw;

	tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

	//ROS_INFO("Euler Angles: (%.3f, %.3f, %.3f)", roll, pitch, yaw);
	return (float)yaw;
}

void syncedCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
{

	if(current_index >= NUM_GRAPH_POINTS)return;
//	ROS_INFO("We are synchronized %s, %s", scan_msg->header.frame_id.c_str(), odom_msg->header.frame_id.c_str());
	geometry_msgs::Point p = odom_msg->pose.pose.position;
	if(current_index == 0)
	{
		ROS_INFO("%d: (%f, %f, %f)", current_index, p.x, p.y, p.z);

		scanTempArray[0] = *scan_msg;
		x_pos[0] = p.x;
		y_pos[0] = p.y;
		yaw[0] = getYaw(odom_msg->pose.pose.orientation);
		current_index++;
	}else {
		if(std::abs(p.x - x_pos[current_index - 1]) >= DIST_BETWEEN_POINTS || std::abs(p.y - y_pos[current_index - 1]) >= DIST_BETWEEN_POINTS )
		{
		ROS_INFO("%d: (%f, %f, %f)", current_index, p.x, p.y, p.z);
			scanTempArray[current_index] = *scan_msg;
			x_pos[current_index] = p.x;
			y_pos[current_index] = p.y;
			yaw[current_index] = getYaw(odom_msg->pose.pose.orientation);
			current_index++;
		}
	}

}

#if USE_FIRST
void first_occupancy_gridmap_test(ros::Publisher& globalmap_pub, float sensor_range)
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
			float ya = (float)y * 0.05f - (500.0f * 0.05f); //to world coordinate
			for(int x = 0; x < 1000; x++)
			{
				float xa = (float)x * 0.05f - (500.0f * 0.05f); //to world coordinate
				int index = x + y * 1000;
				float cell_value = 0;
				int num_reaching_cell = 0;
				for(int i = 0; i < 10; i++)
				{
					float dx = xa - x_pos[i];
					float dy = ya - y_pos[i];
					float distance_from_sensor_reading = std::sqrt(dx*dx + dy*dy);
					//ROS_INFO("Dist %f | (%f, %f)", distance_from_sensor_reading, xa, ya);
					if(distance_from_sensor_reading <= sensor_range)
					{
						//ROS_INFO("Using Sensor %d", i);
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
							continue; //unkown behind obstacle
						}

						cell_value += sensor_value;
						num_reaching_cell++;
					}
				}
				if(num_reaching_cell > 0)
				{
					cell_value /= num_reaching_cell;
					//ROS_INFO("%d Cell Value: %f, %d, %d", index, cell_value, cell_value, num_reaching_cell);
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
}
#elif USE_RAY_TRACE

void ray_trace_occupancy_gridmap(ros::Publisher& gridmap_pub)
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
	ROS_INFO("Generating globalmap from graph");

	//For debugging only use obstacle iformation within 10m radius.
	float sensor_range = 10.0f / MAP_RESOLUTION;


	//Loop through all points in graph
	for(int k = 0; k < NUM_GRAPH_POINTS; k++)
	{
		ROS_INFO("Ray tracing for sensor %d", k);
		sensor_msgs::LaserScan& scan = scanTempArray[k];
		float angle_range = (scan.angle_max - scan.angle_min);
		int num_angles = (angle_range)/scan.angle_increment;

		//Ray trace for sensor k
		for(int i = 0; i < num_angles; i++)
	        {
        	        float xDist = 0;
                	float yDist = 0;

			//@Todo: adjust with position of scan
			int x_start = x0 + x_pos[k] / MAP_RESOLUTION;
			int y_start = y0 + y_pos[k] / MAP_RESOLUTION;

	                int xa = x_start;
	                int ya = y_start;
			//take into account the current rotation of the robot(yaw)
	                float angle = yaw[k] + scan.angle_min + i * scan.angle_increment;
			float dy = sin(angle);
                	float dx = cos(angle);

                	float obstacle_dist = scan.ranges[i]/MAP_RESOLUTION;

	                while(xa >= 0 && xa < MAP_WIDTH && ya >= 0 && ya < MAP_HEIGHT)
        	        {
				int index = xa + ya * globalmap_msg.info.width;
                        	float dist = std::sqrt(xDist*xDist + yDist*yDist);

				if(dist >= sensor_range)break;

                       		if(floor(dist) >= floor(obstacle_dist))
                        	{
					//@Todo: combine with value in cell
                                	if(num_for_average.data[index] == 0)
						globalmap_msg.data[index] = 1;
					else
						globalmap_msg.data[index] += 1;
					num_for_average.data[index] += 1;

                                	//this ray has reached it's destination, go to next ray.
					break;
                        	}
				//@Todo: combine with value in cell
				if(num_for_average.data[index] == 0)
					globalmap_msg.data[index] = 0;
				else
				num_for_average.data[index] += 1;

				xDist += dx;
                        	yDist += dy;

                        	xa = x_start + xDist;
                        	ya = y_start + yDist;
                	}

	        }
	}

	//compute average of sensor information for each cell
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

#elif USE_RAY_TRACE_LOG_ODDS

#endif

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
		if(current_index == NUM_GRAPH_POINTS)
		{
#if USE_FIRST
			first_occupancy_gridmap_test(globalmap_pub, sensor_range);
#elif USE_RAY_TRACE
			ray_trace_occupancy_gridmap(globalmap_pub);
#elif USE_RAY_TRACE_LOG_ODDS
#endif
			current_index++;
		}
		ros::spinOnce();
	}
	return 0;
}
