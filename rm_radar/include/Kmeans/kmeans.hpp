//
// Created by ywj on 24-6-22.
//

#ifndef KMEANS_H
#define KMEANS_H

#include <iostream>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <random>

struct PointUV : public cv::Point3f
{
  float u;
  float v;
  float depth;
};

struct PointXYZ_UV
{
  pcl::PointXYZ xyz;
  PointUV uv;
};

class KMeans
{
private:
  unsigned int max_iteration_;
  const unsigned int cluster_num_;//k

  static double pointsDist(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
  {
    //std::cerr << p1.x<<std::endl;
    //三维空间中的欧氏距离计算公式
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }

  static void compute3DCentroid_PointUV(std::vector<float>& cloud_iterator, float& centroid)
  {
    size_t num = cloud_iterator.size();
    for (const auto& p : cloud_iterator)
      centroid += p;
    centroid /= (float)num;
  }

  static void compute3DCentroid_PointUV(std::vector<PointUV>& cloud_iterator, PointUV& centroid)
  {
    size_t num = cloud_iterator.size();
    for (const auto& p : cloud_iterator)
    {
      centroid.u += p.u;
      centroid.v += p.v;
      centroid.depth += p.depth;
    }
    centroid.u /= (float)num;
    centroid.v /= (float)num;
    centroid.depth /= (float)num;
  }

public:
  pcl::PointCloud<pcl::PointXYZ> centre_points_;
  //KMeans() = default;
  KMeans(unsigned int k, unsigned int max_iteration) :cluster_num_(k), max_iteration_(max_iteration){}

  void init_centroids(pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    centre_points_.clear();
    centre_points_ = kmeans_plus(cloud);
  }

  pcl::PointCloud<pcl::PointXYZ> kmeans_plus(pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    centre_points_.resize(cluster_num_);

    centre_points_[0] = cloud.at(0);


    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> kMeans(const std::vector<PointXYZ_UV>& cloud)
  {
    if (!cloud.empty() && !centre_points_.empty())
    {
      unsigned int iterations = 0;
      double sum_diff = 0.2;
      std::vector<pcl::PointCloud<pcl::PointXYZ>>cluster_cloud;
      while (!(iterations >= max_iteration_ || sum_diff <= 0.05))//如果大于迭代次数或者两次重心之差小于0.05就停止
      {
        sum_diff = 0;
        cluster_cloud.clear();
        cluster_cloud.resize(cluster_num_);
        for (const auto& point : cloud)
        {
          std::vector<double>dists(0, 0);
          for (size_t j = 0; j < cluster_num_; ++j)
          {
            dists.emplace_back(pointsDist(point.xyz, centre_points_.points[j]));
          }
          auto min_dist = std::min_element(dists.cbegin(), dists.cend());
          unsigned int it = std::distance(dists.cbegin(), min_dist);//获取最小值所在的序号或者位置（从0开始）
          cluster_cloud[it].points.push_back(point.xyz);//放进最小距离所在的簇
        }
        //重新计算簇重心
        pcl::PointCloud<pcl::PointXYZ> new_centre;
        for (size_t k = 0; k < cluster_num_; ++k)
        {
          Eigen::Vector4f centroid;
          pcl::PointXYZ centre;
          pcl::compute3DCentroid(cluster_cloud[k], centroid);
          centre.x = centroid[0];
          centre.y = centroid[1];
          centre.z = centroid[2];
          //centre_points_->clear();
          //centre_points_->points.push_back(centre);
          new_centre.points.push_back(centre);

        }
        //计算重心变化量
        for (size_t s = 0; s < cluster_num_; ++s)
        {
          sum_diff += pointsDist(new_centre.points[s], centre_points_.points[s]);
        }
        centre_points_.clear();
        centre_points_ = new_centre;

        ++iterations;
      }
      return centre_points_;
    }
    return {};
  }

  ~KMeans() = default;
};


#endif  // KMEANS_H
