#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;

ros::Publisher cloud_pub;
tf::TransformListener* listener;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PoseStampedConstPtr& pose) {
	sensor_msgs::PointCloud2 cloud_transformed;
	pcl_ros::transformPointCloud(pose->header.frame_id, *cloud_msg, cloud_transformed, *listener);
	
	PointCloud cloud;
	pcl::fromROSMsg(cloud_transformed, cloud);
	
	float resolution = 0.02f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
	octree.setInputCloud(cloud.makeShared());
	octree.addPointsFromInputCloud();
	
	if(!((pose->pose.position.x == 0) && (pose->pose.position.y) == 0 && (pose->pose.position.z == 0))) {
		ROS_WARN("Expecting point to be the origin of its coordinate frame, but it isn't");
		//TODO broadcast the frame corresponding to this pose?
	}
	Eigen::Vector3f    origin(pose->pose.position.x,        pose->pose.position.y, pose->pose.position.z);
	Eigen::Vector3f direction(pose->pose.position.x + RAND_MAX, pose->pose.position.y + 0.0f, pose->pose.position.z + 0.0f);
	//AlignedPointTVector voxelCenterList;
	std::vector<int> k_indices;
	
	//int nPoints = octree.getIntersectedVoxelCenters(origin, direction, voxelCenterList);
	int nPoints = octree.getIntersectedVoxelIndices(origin, direction, k_indices);
	ROS_INFO("vector intersected %d voxels", nPoints);
	
	
	PointCloud::Ptr out (new PointCloud);
	out->header.stamp = ros::Time::now();
	out->header.frame_id = pose->header.frame_id;
	for(int i=0; i<nPoints; i++) {
		out->points.push_back(cloud.points[k_indices[i]]);
		ROS_INFO("%d: Point is: x=%f, y=%f, z=%f", k_indices[i], cloud.points[k_indices[i]].x, cloud.points[k_indices[i]].y, cloud.points[k_indices[i]].z);

	}
	cloud_pub.publish(out);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "world_intersect");
	ros::NodeHandle nh;
	listener = new tf::TransformListener();
    cloud_pub = nh.advertise<PointCloud>("intersected_points", 1);
	// ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	// ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pt_in", 1, ptCallback);
	message_filters::Subscriber<sensor_msgs::PointCloud2>   cloud_sub(nh, "cloud", 1);
	message_filters::Subscriber<geometry_msgs::PoseStamped>   pose_sub(nh, "pose", 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproximatePolicy;
	message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), cloud_sub, pose_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
		
	ros::spin();
}