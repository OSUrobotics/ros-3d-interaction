/*
 * Copyright (c) 2013, Oregon State University
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Oregon State University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author Dan Lazewatsky/lazewatd@engr.orst.edu
 */
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <float.h>
#include <math.h>


template <typename T>
class NeverEmptyQueue : public std::queue<T>
{
	public:
		virtual void pop() {
			if(this->size() > 1) std::queue<T>::pop();
		} 
};


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;

ros::Publisher cloud_pub;
ros::Publisher point_pub;
tf::TransformListener* listener;
tf::TransformBroadcaster* broadcaster;
tf::StampedTransform g_transform;
bool g_transform_ready = false;

sensor_msgs::PointCloud2ConstPtr   g_cloud;
geometry_msgs::PoseStampedConstPtr g_pose;

bool g_cloud_ready = false;
bool g_pose_ready  = false;

NeverEmptyQueue<sensor_msgs::PointCloud2ConstPtr> g_cloud_queue;
NeverEmptyQueue<geometry_msgs::PoseStampedConstPtr> g_pose_queue;

// params
double g_resolution, g_min_dist;
std::string g_vector_frame;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
	g_cloud_queue.push(cloud_msg);
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
	g_pose_queue.push(pose);
}

// void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PoseStampedConstPtr& pose) {
void intersectPose(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PoseStampedConstPtr& pose) {
	sensor_msgs::PointCloud2 cloud_transformed;
	std::string vector_frame = pose->header.frame_id;
	ros::Time stamp = ros::Time(0);//(cloud_msg->header.stamp > pose->header.stamp) ? cloud_msg->header.stamp : pose->header.stamp;
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
		g_transform = tf::StampedTransform(trans, stamp, pose->header.frame_id, vector_frame);
		g_transform_ready = true;
		// broadcaster->sendTransform(tf::StampedTransform(trans, stamp, pose->header.frame_id, vector_frame));
	}
	sensor_msgs::PointCloud2 cloud_msg_copy = *cloud_msg;
	cloud_msg_copy.header.stamp=ros::Time(0);
	listener->waitForTransform(cloud_msg->header.frame_id, vector_frame, stamp, ros::Duration(2.0));
	pcl_ros::transformPointCloud(vector_frame, cloud_msg_copy, cloud_transformed, *listener);
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
	PointCloud::Ptr out (new PointCloud);
	out->header.frame_id = vector_frame;
	for(int i=0; i<nPoints; i++) {
		if(g_min_dist < DBL_MAX) {
			pcl::PointXYZ pt = cloud.points[k_indices[i]];
			if(sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) > g_min_dist)
				out->points.push_back(cloud.points[k_indices[i]]);
		}
	}
	out->header.stamp = ros::Time::now();
	sensor_msgs::PointCloud2 out_msg;
	pcl::toROSMsg(*out, out_msg); 	
	sensor_msgs::PointCloud2 out_transformed;
	out_msg.header.stamp=ros::Time(0);
	listener->waitForTransform(out->header.frame_id, "base_link", ros::Time(0), ros::Duration(2.0));
	pcl_ros::transformPointCloud("base_link", out_msg, out_transformed, *listener);
	cloud_pub.publish(out_transformed);
}

void sendTransform(const ros::TimerEvent& te) {
	if(g_transform_ready) {
		g_transform.stamp_ = ros::Time::now();
		broadcaster->sendTransform(g_transform);
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "world_intersect");
	ros::NodeHandle nh;
	
	ros::NodeHandle pnh("~");
	pnh.param("vector_frame",      g_vector_frame, std::string("intersected_vector"));
	pnh.param("octree_resolution", g_resolution,   0.02);
	pnh.param("min_dist",          g_min_dist,     DBL_MAX);
	
	listener = new tf::TransformListener(ros::Duration(45));
	broadcaster = new tf::TransformBroadcaster();
    //cloud_pub = nh.advertise<PointCloud>("intersected_points", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("intersected_points", 1);
    point_pub = nh.advertise<geometry_msgs::PointStamped>("intersected_point", 1);

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCallback);
	
	ros::Rate rate(10);
	while(((g_cloud_queue.size() == 0) || (g_pose_queue.size() == 0)) && ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	ros::Timer timer = nh.createTimer(ros::Duration(0.05), sendTransform, false);

	while(ros::ok()) {
		//sensor_msgs::PointCloud2ConstPtr& c = g_cloud_queue.front();
		intersectPose(g_cloud_queue.front(), g_pose_queue.front());
		g_cloud_queue.pop();
		g_pose_queue.pop();
		ros::spinOnce();
		rate.sleep();
	}
	
	// message_filters::Subscriber<sensor_msgs::PointCloud2>   cloud_sub(nh, "cloud", 1);
	// message_filters::Subscriber<geometry_msgs::PoseStamped>   pose_sub(nh, "pose", 1);
	// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproximatePolicy;
	// message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), cloud_sub, pose_sub);
	// sync.registerCallback(boost::bind(&intersectPose, _1, _2));
		
	//ros::spin();
}
