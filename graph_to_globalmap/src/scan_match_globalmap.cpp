#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

typedef Eigen::MatrixXf MAT;

#define PUBLISH_LOCAL_MAPS 0

//Map info
#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define MAP_RESOLUTION 0.05f

//Scan collection
#define NUM_GRAPH_POINTS 10
#define DIST_BETWEEN_POINTS 0.05f

//Macros
#define SQUARED(x) x*x

//Topics
#define LASERSCAN_TOPIC "scan"
#define POINTCLOUD_TOPIC "os1_cloud_node/points"
#define ODOMETRY_TOPIC "odom"
#define GLOBALMAP_TOPIC "globalmap"

typedef struct {
	float dx;
	float dy;
	float yaw;
}Position;


//@Todo: This stores duplicate information data. We only need to store that information once. Then we should store only the actual sensor data for each node.
sensor_msgs::LaserScan* sensor_data = new sensor_msgs::LaserScan[NUM_GRAPH_POINTS];
sensor_msgs::PointCloud2* sensor3D_data = new sensor_msgs::PointCloud2[NUM_GRAPH_POINTS];
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

//const Position scanMatchICP(sensor_msgs::PointCloud2& prev_scan, float prev_yaw, const sensor_msgs::PointCloud2::ConstPtr new_scan, float dx, float dy, float new_yaw)
const Position scanMatchICP(sensor_msgs::PointCloud2& prev_scan, float prev_yaw, sensor_msgs::PointCloud2& new_scan, float dx, float dy, float new_yaw)
{
	//@Todo: implement ICP here

	// Get center of mass
	MAT u_x(3, 1); u_x << 0,0,0;
	MAT u_p(3, 1); u_p << 0,0,0;
	int num_rays = 0;

	for(sensor_msgs::PointCloud2Iterator<float> iter_x(prev_scan, "x"), iter_p(new_scan, "x"); iter_x != iter_x.end(); ++iter_x, ++iter_p)
	{
		MAT x(3, 1); x << iter_x[0], iter_x[1], iter_x[2];
		MAT p(3, 1); p << iter_p[0], iter_p[1], iter_p[2];
		u_x += x;
		u_p += p;

		num_rays++;
	}

	if(num_rays != 0)
	{
		u_x /= (float)num_rays;
		u_p /= (float)num_rays;
	}
	//ROS_INFO("u_x mean: (%.3f, %.3f, %.3f)", u_x[0], u_x[1], u_x[2]);
	//ROS_INFO("u_p mean: (%.3f, %.3f, %.3f)", u_p[0], u_p[1], u_p[2]);


	//1) find the closest point in prev_scan for each point in new_scan, (offset new scan by dx and dy)

	// Shift by center of mass
	//2) minimize the distance error between each point.
	MAT W(3, 3); W <<	0, 0, 0,	0, 0, 0,	0, 0, 0;
	for(sensor_msgs::PointCloud2Iterator<float> iter_x(prev_scan, "x"), iter_p(new_scan, "x"); iter_x != iter_x.end(); ++iter_x, ++iter_p)
	{
		MAT xp(3, 1); xp << iter_x[0], iter_x[1], iter_x[2];
		MAT pp(3, 1); pp << iter_p[0], iter_p[1], iter_p[2];
	
		xp -= u_x;
		pp -= u_p;

		MAT Wn(3, 3); Wn = xp * pp.transpose();
		W += Wn;
	}
	Eigen::JacobiSVD<MAT> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);

#if 0	//DEBUG SVD OUTPUT
	std::cout << "W: " << W << "\n";
	
	std::cout << "Its singular values are:\n" << svd.singularValues() << "\n";
	std::cout << "Its left singular vectors are the columns of the thin U matrix:\n" << svd.matrixU() << "\n";
	std::cout << "Its right singular vectors are the columns of the thin V matrix:\n" << svd.matrixV() << "\n";
#endif	

	MAT R(3,3);
	R = svd.matrixU() * svd.matrixV().transpose();
	MAT t(3, 1);
	t = u_x - R*u_p;

	Eigen::Matrix4f Trans; // Your Transformation Matrix
	Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	Trans.block<3,3>(0,0) = R;
	Trans.block<3,1>(0,3) = t;

	  // Executing the transformation

	  
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	// You can either apply transform_1 or transform_2; they are the same

	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(new_scan, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
	pcl::transformPointCloud (*temp_cloud, *transformed_cloud, Trans);
	
	ROS_INFO("Apply transformation");

	

	//return new yaw and dx, dy
	Position pos;
	pos.dx = dx;
	pos.dy = dy;
	pos.yaw = new_yaw;
	return pos;
}


void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const sensor_msgs::PointCloud2::ConstPtr& scan3D_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	if(scan_index >= NUM_GRAPH_POINTS)return;

	static float prev_x = 0;
	static float prev_y = 0;
	static float prev_yaw = 0;

	geometry_msgs::Point p = odom_msg->pose.pose.position;

	//Calculate relative position
	float dx = prev_x - p.x;
	float dy = prev_y - p.y;

	//Using distance squared. Costly to use square root if we don't need to.
	float distance_squared = dx*dx + dy*dy;

	//@Note: This won't save the first position. It will start saving from a distance NUM_GRAPH_POINTS away from (0,0)
	if(distance_squared >= SQUARED(DIST_BETWEEN_POINTS))
	{
		//ROS_INFO("%d | odometry (%f, %f) | relative position (%f, %f)", scan_index, p.x, p.y, dx, dy);
		//ROS_INFO("height %d | width %d | point_bytes %d | row_bytes %d | row %d", scan3D_msg->height, scan3D_msg->width, scan3D_msg->point_step, scan3D_msg->row_step, scan3D_msg->row_step/scan3D_msg->point_step);

		sensor_msgs::PointCloud2 new_cloud = *scan3D_msg;
		static int temp_once = 0;
		if (temp_once++ == 0)
		{
			int zero_counter = 0;
			int counter = 0;
			/*
			for(sensor_msgs::PointCloud2Iterator<float> iter_x(new_cloud, "x"), iter_y(new_cloud, "y"), iter_z(new_cloud, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
			{
				if(counter++ % 4 != 0)continue;
				if (std::isnan(iter_x[0]) || std::isnan(iter_y[0]) || std::isnan(iter_z[0]))
				{
					ROS_INFO("rejected for nan in point(%f, %f, %f)\n", iter_x[0], iter_y[0], iter_z[0]);
					continue;
				}

				if(iter_x[0] == 0 && iter_y[0] == 0 && iter_z[0] == 0) zero_counter++;
				//if(counter % scan3D_msg->row_step / scan3D_msg->point_step == 0)
					//ROS_INFO("%d: (%f, %f, %f)", counter, iter_x[0], iter_y[0], iter_z[0]);
				if(counter >= scan3D_msg->row_step / scan3D_msg->point_step)break;
			}
			*/
			ROS_INFO("zero_counter %d", zero_counter);
			/*
			ROS_INFO("int_byte_size %d", sizeof(int));
			ROS_INFO("iterate: %d", scan3D_msg->point_step/sizeof(int));
			for(int i = 0; i < scan3D_msg->point_step; i++)
			{
				if(i%4 == 0)
					value = 0;
				if(scan3D_msg->is_bigendian)
				{
					//value |= scan3D_msg->data[i] << (24 - (i%4)*8);
					((char*)&value)[3 - i%4] = scan3D_msg->data[i];
				}else
				{
					//value |= scan3D_msg->data[i] << (i%4)*8;
					((char*)&value)[i%4] = scan3D_msg->data[i];
				}
				if(i%4 == 3)
					ROS_INFO("%d - %d: value %f", temp2, i, value);
			}
			*/
		}
//Save position to compute relative distance for next scan
		prev_x = p.x;
		prev_y = p.y;

		//Store laser scan
		sensor_data[scan_index] = *scan_msg;
		sensor3D_data[scan_index] = *scan3D_msg;

		//Store relative position
		if(scan_index == 0)
		{
			relative_positions[scan_index].dx = dx;
			relative_positions[scan_index].dy = dy;
			relative_positions[scan_index].yaw = getYaw(odom_msg->pose.pose.orientation);
		}
		else
		{
			relative_positions[scan_index] = scanMatchICP(sensor3D_data[scan_index-1], prev_yaw, sensor3D_data[scan_index], dx, dy, getYaw(odom_msg->pose.pose.orientation));
			//relative_positions[scan_index] = scanMatchICP(boost::make_shared<sensor_msgs::PointCloud2 const>(sensor3D_data[scan_index-1]), prev_yaw, scan3D_msg, dx, dy, getYaw(odom_msg->pose.pose.orientation));
		}
		//Store previous orientation
		prev_yaw = relative_positions[scan_index].yaw;

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
	message_filters::Subscriber<sensor_msgs::PointCloud2> sensor3D_sub(node_handle, POINTCLOUD_TOPIC, 1);
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

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::PointCloud2, nav_msgs::Odometry> mypolicy;
	message_filters::Synchronizer<mypolicy> sync(mypolicy(10), sensor_sub, sensor3D_sub, odom_sub);

	sync.registerCallback(boost::bind(&scanOdomCallback, _1, _2, _3));
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
	delete[] sensor3D_data;
	delete[] relative_positions;

	return 0;
}
