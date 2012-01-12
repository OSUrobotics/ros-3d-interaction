#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace laser_assembler;

ros::ServiceClient client;

PointCloud::Ptr getLaserCloud() {
	PointCloud::Ptr cloud (new PointCloud);
	AssembleScans srv;
	srv.request.begin = ros::Time(0,0);
	srv.request.end   = ros::Time::now();
	if (client.call(srv)) {
		for(int i=0; i<srv.response.cloud.points.size(); i++) {
			pcl::PointXYZ p = pcl::PointXYZ();
			p.x = srv.response.cloud.points[i].x;
			p.y = srv.response.cloud.points[i].y;
			p.z = srv.response.cloud.points[i].z;
			
			cloud->push_back(p);
		}
	} else ROS_ERROR("Service call failed\n");
	return cloud;
}

void cloudCallback(const PointCloud::ConstPtr& cloud1) {
	PointCloud::Ptr cloud2 = getLaserCloud();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud1);
	icp.setInputTarget(cloud2);
	PointCloud Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	          icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;	
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "find_transform");
	ros::NodeHandle nh;
	ROS_INFO("Waiting for assembled tilt scans");
	ros::service::waitForService("assemble_scans");
	client = nh.serviceClient<AssembleScans>("assemble_scans");
	ros::Subscriber cloud_sub = nh.subscribe<PointCloud>("cloud", 1, cloudCallback);
	
	
	ros::spin();
	return 0;
}