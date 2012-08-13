#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cmvision/Blobs.h>
#include <cmvision/Blob.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>

bool g_cloud_ready = false;
bool g_blobs_ready = false;
std::string g_cloud_frame;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud depth;
cmvision::Blobs blobs;

cv_bridge::CvImagePtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	//pcl::fromROSMsg(*msg, cloud);
	
	g_cloud_frame = msg->header.frame_id;
	g_cloud_ready = true;
}

void blobCallback(const cmvision::Blobs::ConstPtr& msg) {
	blobs = *msg;
	g_blobs_ready = true;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "blobs_2_3d");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);
	//ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	image_transport::Subscriber depth_sub = it.subscribe("image", 1, depthCallback);
	ros::Subscriber blob_sub  = nh.subscribe<cmvision::Blobs>("blobs", 1, blobCallback);
	
	ros::Publisher cloud_pub = nh.advertise<PointCloud>("blob_cloud", 1);
		
	ros::Rate rate(10);	
	while(ros::ok()) {
		PointCloud::Ptr out (new PointCloud);
		out->header.stamp = ros::Time::now();
		out->header.frame_id = g_cloud_frame;
		ros::spinOnce();
		BOOST_FOREACH(cmvision::Blob blob, blobs.blobs) {
			out->points.push_back(cloud.at(blob.x, blob.y));
		}
		cloud_pub.publish(out);
		rate.sleep();
	}	
	return 0;
}
