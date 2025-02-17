#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZI;
using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_non_ground;

template <typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.stamp = stamp;
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr& cloud_msg)
{
  double time_taken;

  pcl::PointCloud<PointType> pc_curr;
  pcl::PointCloud<PointType> pc_ground;
  pcl::PointCloud<PointType> pc_non_ground;

  pcl::fromROSMsg(*cloud_msg, pc_curr);

  PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

  pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Demo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string cloud_topic = "/livox/lidar";
  //    pnh.param<string>("cloud_topic", cloud_topic, "/pointcloud");

  cout << "Operating patchwork++..." << endl;
  PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));

  pub_non_ground = pnh.advertise<sensor_msgs::PointCloud2>("/livox/lidar/nonground", 10, true);

  ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

  ros::spin();

  return 0;
}
