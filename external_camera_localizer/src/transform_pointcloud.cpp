#include <iostream>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


ros::Publisher cloud_pub;
std::string g_target_frame;
tf::TransformListener* listener;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	sensor_msgs::PointCloud2 out;
	pcl_ros::transformPointCloud(g_target_frame, *cloud, out, *listener);
	cloud_pub.publish(out);
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "transform_pointcloud");
	ros::NodeHandle nh;
	ros::NodeHandle nhl("~");
	nhl.param("target_frame", g_target_frame, std::string("/map"));
	//ros::service::waitForService("assemble_scans");
	//client = nh.serviceClient<AssembleScans>("assemble_scans");
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_transformed", 1);
	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	listener = new tf::TransformListener();
	
	ros::spin();
	return 0;
}