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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

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

const Position scanMatchICP(sensor_msgs::PointCloud2& prev_scan, float prev_yaw, sensor_msgs::PointCloud2& new_scan, float dx, float dy, float new_yaw)
{

	#if 1
	#define DEBUG_ICP(x) x
	#else
	#define DEBUG_ICP(x)
	#endif

	DEBUG_ICP(ROS_INFO("\n\n-----------ICP computation-----------");)

	//Convert to pcl point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_scan_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_scan_pcl (new pcl::PointCloud<pcl::PointXYZ> ());
#define VOXEL_SIZE 1.0f
	{
		pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
		pcl::PCLPointCloud2::Ptr filtered_temp_cloud (new pcl::PCLPointCloud2 ());
		pcl_conversions::toPCL(prev_scan, *temp_cloud);

		// Create the filtering object
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
		voxel_filter.setInputCloud(temp_cloud);
		voxel_filter.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
		voxel_filter.filter(*filtered_temp_cloud);

		pcl::fromPCLPointCloud2(*filtered_temp_cloud, *prev_scan_pcl);
	}

	{
		pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
		pcl::PCLPointCloud2::Ptr filtered_temp_cloud (new pcl::PCLPointCloud2 ());
		pcl_conversions::toPCL(new_scan, *temp_cloud);

		// Create the filtering object
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
		voxel_filter.setInputCloud(temp_cloud);
		voxel_filter.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
		voxel_filter.filter(*filtered_temp_cloud);

		pcl::fromPCLPointCloud2(*filtered_temp_cloud, *new_scan_pcl);

		new_scan_pcl->is_dense = false;
	}
	DEBUG_ICP(ROS_INFO("Converted PointCloud2 to pcl::PointXYZ");)

	//Setup data for ordered scan
	pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_scan (new pcl::PointCloud<pcl::PointXYZ> ());
	ordered_scan->width  = new_scan_pcl->width;
	ordered_scan->height  = new_scan_pcl->height;
	ordered_scan->points.resize(ordered_scan->width * ordered_scan->height);

	int cloud_size = prev_scan_pcl->points.size();
	if(new_scan_pcl->points.size() < cloud_size)cloud_size = new_scan_pcl->points.size();
	DEBUG_ICP(ROS_INFO("Cloud size:%d", cloud_size);)

	//Iterate
	bool icp_done = false;
	int iterations = 0;
	pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
	int K = 10; //For K-nearest neighbor
	std::vector<int> search_points_index(K);
	std::vector<float> search_squared_distance(K);
	DEBUG_ICP(ROS_INFO("Setup kdTree");)


	
	//Shift by center of mass
	DEBUG_ICP(ROS_INFO("Shift by center of mass");)
	//Prev scan shifting
	MAT mean_prev_scan(3, 1); mean_prev_scan << 0,0,0;
	int size_prev = prev_scan_pcl->points.size();
	for(int i = 0; i < size_prev; i++)
	{
		//sum all points in cloud to compute center of mass
		MAT point_prev_scan(3, 1); point_prev_scan << prev_scan_pcl->points[i].x, prev_scan_pcl->points[i].y, prev_scan_pcl->points[i].z;
		mean_prev_scan += point_prev_scan;
	}
	mean_prev_scan /= (float)size_prev;

	for(int i = 0; i < size_prev; i++)
	{
		pcl::PointXYZ& p = prev_scan_pcl->points[i];
		p.x -= mean_prev_scan(0,0);
		p.y -= mean_prev_scan(1,0);
		p.z -= mean_prev_scan(2,0);
	}
	
	//new scan shifting
	MAT mean_new_scan(3, 1); mean_new_scan << 0,0,0;
	int size_new = new_scan_pcl->points.size();
	for(int i = 0; i < size_new; i++)
	{
		//sum all points in cloud to compute center of mass
		MAT point_new_scan(3, 1); point_new_scan << new_scan_pcl->points[i].x, new_scan_pcl->points[i].y, new_scan_pcl->points[i].z;
		mean_new_scan += point_new_scan;
	}	
	
	mean_new_scan /= (float)size_new;

	for(int i = 0; i < size_new; i++)
	{
		pcl::PointXYZ& p = new_scan_pcl->points[i];
		p.x -= mean_new_scan(0,0);
		p.y -= mean_new_scan(1,0);
		p.z -= mean_new_scan(2,0);
	}


	while (!icp_done)
	{
		DEBUG_ICP(ROS_INFO("ICP iteration: %d", iterations);)

		//Order new_scan_pcl realtive to the nearest nighbor in prev_scan_pcl
		kd_tree.setInputCloud(new_scan_pcl);
		DEBUG_ICP(ROS_INFO("Sorting new scan");)
		for(int i = 0; i < cloud_size; i++)
		{
			pcl::PointXYZ& search_point = prev_scan_pcl->points[i];

			//find nearest point in new_scan relative to search point
			if(kd_tree.nearestKSearch(search_point, K, search_points_index, search_squared_distance) > 0)
			{
				int nearest_index = search_points_index[0];
				float shortest_distance = search_squared_distance[0];
				for(int b = 1; b < search_points_index.size(); b++)
				{
					if(search_squared_distance[b] < shortest_distance)
					{
						nearest_index = search_points_index[b];
						shortest_distance = search_squared_distance[b];
					}
				}
				ordered_scan->points[i] = new_scan_pcl->points[nearest_index];
			}
			else
			{
				ordered_scan->points[i] = search_point;
				ROS_INFO("ERROR: Couldn't find closest point");
			}
		}
		search_points_index.clear(); search_squared_distance.clear();//Cleanup for next iteration
		new_scan_pcl.swap(ordered_scan);

		DEBUG_ICP(
		for(int i = 0; i < cloud_size; i++)
		{
			if(i % 100 == 0)
				ROS_INFO("POINT %d: (%f, %f, %f))", i, new_scan_pcl->points[i].x,  new_scan_pcl->points[i].y,  new_scan_pcl->points[i].z);
		}
		)

		//Compute Transformation
		DEBUG_ICP(ROS_INFO("Compute transformation");)
		MAT W(3, 3); W <<	0, 0, 0,	0, 0, 0,	0, 0, 0;
		for(int i = 0; i < cloud_size; i++)
		{
			MAT point_prev_scan(3, 1); point_prev_scan << prev_scan_pcl->points[i].x, prev_scan_pcl->points[i].y, prev_scan_pcl->points[i].z;
			MAT point_new_scan(3, 1); point_new_scan << new_scan_pcl->points[i].x, new_scan_pcl->points[i].y, new_scan_pcl->points[i].z;

			MAT Wn(3, 3); Wn = point_prev_scan * point_new_scan.transpose();
			W += Wn;
		}
		Eigen::JacobiSVD<MAT> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);


		MAT rotation(3,3);
		rotation = svd.matrixU() * svd.matrixV().transpose();
		MAT translation(3, 1);
		translation = mean_prev_scan - rotation * mean_new_scan;


		Eigen::Matrix4f transform_matrix;
		transform_matrix.setIdentity();	// Set to identity matrix
		transform_matrix.block<3,3>(0,0) = rotation;
		transform_matrix.block<3,1>(0,3) = translation;

		Eigen::Ref<Eigen::Matrix3f> rot3x3(rotation);
		Eigen::Vector3f euler_angles = rot3x3.eulerAngles(0, 1, 2);
		DEBUG_ICP(ROS_INFO("Euler angles: %f, %f, %f", euler_angles[0], euler_angles[1], euler_angles[2]);)
		DEBUG_ICP(ROS_INFO("Translation: %f, %f, %f", translation(0, 0), translation(1, 0), translation(2, 0));)

		//Apply transformation
		DEBUG_ICP(ROS_INFO("Apply transfomation");)
		ordered_scan->points.clear();
		pcl::transformPointCloud (*new_scan_pcl, *ordered_scan, transform_matrix);
		new_scan_pcl.swap(ordered_scan);


		//@TODO: Apply transformation to center of mass
		//mean_prev_scan
		//mean_new_scan


		DEBUG_ICP(ROS_INFO("Transformation applied");)

		//TODO: Store applied transformation
		float error = 0;
		for(int i = 0; i < cloud_size; i++)
		{
			MAT point_prev_scan(3, 1); point_prev_scan << prev_scan_pcl->points[i].x, prev_scan_pcl->points[i].y, prev_scan_pcl->points[i].z;
			MAT point_new_scan(3, 1); point_new_scan << new_scan_pcl->points[i].x, new_scan_pcl->points[i].y, new_scan_pcl->points[i].z;

			float x2 = 
			point_prev_scan(0, 0) * point_prev_scan(0, 0) + 
			point_prev_scan(1, 0) * point_prev_scan(1, 0) +
			point_prev_scan(2, 0) * point_prev_scan(2, 0);


			float y2 = 
			point_new_scan(0, 0) * point_new_scan(0, 0) + 
			point_new_scan(1, 0) * point_new_scan(1, 0) +
			point_new_scan(2, 0) * point_new_scan(2, 0);
			
			error += sqrt(x2) + sqrt(y2) - 2*(svd.singularValues()[0] + svd.singularValues()[1] + svd.singularValues()[2]); 
		}
		ROS_INFO("%f", error);
		//Hack to break icp after fixed num iterations to start with. @TODO: use sum of squared error

		if(++iterations >= 3)
		{
			icp_done = true;
		}
	}


	//return new yaw and dx, dy
	Position pos;
	pos.dx = dx;
	pos.dy = dy;
	pos.yaw = new_yaw;
	DEBUG_ICP(ROS_INFO("\n-----------ICP DONE------------------\n\n");)
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
		ROS_INFO("%d | odometry (%f, %f) | relative position (%f, %f)", scan_index, p.x, p.y, dx, dy);
		//ROS_INFO("height %d | width %d | point_bytes %d | row_bytes %d | row %d", scan3D_msg->height, scan3D_msg->width, scan3D_msg->point_step, scan3D_msg->row_step, scan3D_msg->row_step/scan3D_msg->point_step);

		sensor_msgs::PointCloud2 new_cloud = *scan3D_msg;
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
	ROS_INFO("graph_to_globalmap started");
	while(ros::ok())
	{
		if(scan_index == NUM_GRAPH_POINTS)
		{
		#if PUBLISH_LOCAL_MAPS
			rayTraceGlobalMap(globalmap_pub, local_maps);
		#else
			rayTraceGlobalMap(globalmap_pub);
		#endif

			scan_index++;//Temporary, only generate the map once.
		}
		ros::spinOnce();
	}

	delete[] sensor_data;
	delete[] sensor3D_data;
	delete[] relative_positions;

	return 0;
}
