#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PointStamped.h>

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <vector>
#include <ctime>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
	PointCloud cloud;
	pcl::fromROSMsg(*msg, cloud);
	
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
	octree.setInputCloud(cloud.makeShared());
	octree.addPointsFromInputCloud();
	
}

void ptCallback(const geometry_msgs::PointStamped pt_in) {
	
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "world_intersect");
	ros::NodeHandle nh;
	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	ros::Subscriber pt_sub = nh.subscribe<geometry_msgs::PointStamped>("pt_in", 1, ptCallback);
	ros::spin();
}