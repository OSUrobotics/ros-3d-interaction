#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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
tf::TransformBroadcaster* broadcaster;

// params
double g_resolution;
std::string g_vector_frame;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PoseStampedConstPtr& pose) {
	sensor_msgs::PointCloud2 cloud_transformed;
	std::string vector_frame = pose->header.frame_id;
	ros::Time stamp = (cloud_msg->header.stamp > pose->header.stamp) ? cloud_msg->header.stamp : pose->header.stamp;
	
	if(!((pose->pose.position.x == 0) && (pose->pose.position.y) == 0 && (pose->pose.position.z == 0))) {
		//ROS_WARN("Expecting point to be the origin of its coordinate frame, but it isn't");
		tf::Transform trans;
		trans.setOrigin(tf::Vector3(
			pose->pose.position.x,
			pose->pose.position.y,
			pose->pose.position.z
		));
		trans.setRotation(tf::Quaternion(
			pose->pose.orientation.x,
			pose->pose.orientation.y,
			pose->pose.orientation.z,
			pose->pose.orientation.w
		));
		vector_frame = g_vector_frame;
		broadcaster->sendTransform(tf::StampedTransform(trans, stamp, pose->header.frame_id, vector_frame));
	}
	
	listener->waitForTransform(cloud_msg->header.frame_id, vector_frame, stamp, ros::Duration(2.0));
	pcl_ros::transformPointCloud(vector_frame, *cloud_msg, cloud_transformed, *listener);
	int tPoints = cloud_transformed.width*cloud_transformed.height;
	if(tPoints == 0) return;
	
	PointCloud cloud;
	pcl::fromROSMsg(cloud_transformed, cloud);
	
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (g_resolution);
	octree.setInputCloud(cloud.makeShared());
	octree.addPointsFromInputCloud();
	
	Eigen::Vector3f    origin(0, 0, 0);
	Eigen::Vector3f direction(RAND_MAX, 0.0f, 0.0f);
	std::vector<int> k_indices;
	
	int nPoints = octree.getIntersectedVoxelIndices(origin, direction, k_indices);
	//ROS_INFO("vector intersected %d voxels", nPoints);
	
	PointCloud::Ptr out (new PointCloud);
	out->header.stamp = ros::Time::now();
	out->header.frame_id = vector_frame;
	for(int i=0; i<nPoints; i++) {
		out->points.push_back(cloud.points[k_indices[i]]);
	}
	//ROS_INFO("%d: Point is: x=%f, y=%f, z=%f", k_indices[1], cloud.points[k_indices[1]].x, cloud.points[k_indices[1]].y, cloud.points[k_indices[1]].z);

	cloud_pub.publish(out);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "world_intersect");
	ros::NodeHandle nh;
	
	ros::NodeHandle pnh("~");
	pnh.param("vector_frame",      g_vector_frame, std::string("intersected_vector"));
	pnh.param("octree_resolution", g_resolution,   0.02);
	
	listener = new tf::TransformListener();
	broadcaster = new tf::TransformBroadcaster();
    cloud_pub = nh.advertise<PointCloud>("intersected_points", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2>   cloud_sub(nh, "cloud", 1);
	message_filters::Subscriber<geometry_msgs::PoseStamped>   pose_sub(nh, "pose", 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproximatePolicy;
	message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), cloud_sub, pose_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
		
	ros::spin();
}