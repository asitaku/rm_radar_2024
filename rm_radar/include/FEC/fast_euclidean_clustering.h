//
// Created by ywj on 24-7-11.
//

#ifndef CATKIN_WS_FAST_EUCLIDEAN_CLUSTERING_H
#define CATKIN_WS_FAST_EUCLIDEAN_CLUSTERING_H

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <queue>

#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>


template <typename PointT>
class FastEuclideanClustering : public pcl::PCLBase<PointT> {
  using Base = pcl::PCLBase<PointT>;
  using Base::deinitCompute;
  using Base::indices_;
  using Base::initCompute;
  using Base::input_;

public:
  using KdTree = pcl::search::Search<PointT>;
  using KdTreePtr = typename KdTree::Ptr;
  using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;

  FastEuclideanClustering()
    : cluster_tolerance_(0.0)
    , max_cluster_size_(std::numeric_limits<unsigned int >::max())
    , min_cluster_size_(1)
    , quality_(0.0)
    , tree_()
  {}

  double getClusterTolerance() const
  {
    return cluster_tolerance_;
  }

  void setClusterTolerance(double tolerance)
  {
    cluster_tolerance_ = tolerance;
  }

  unsigned int getMaxClusterSize() const
  {
    return max_cluster_size_;
  }

  void setMaxClusterSize(unsigned int max_cluster_size)
  {
    max_cluster_size_ = max_cluster_size;
  }

  unsigned int getMinClusterSize() const
  {
    return min_cluster_size_;
  }

  void setMinClusterSize(unsigned int min_cluster_size)
  {
    min_cluster_size_ = min_cluster_size;
  }

  double getQuality() const
  {
    return quality_;
  }

  void setQuality(double quality)
  {
    quality_ = quality;
  }

  KdTreePtr getSearchMethod() const
  {
    return (tree_);
  }

  void setSearchMethod(const KdTreePtr& tree)
  {
    tree_ = tree;
  }

  void segment(std::vector<pcl::PointIndices>& clusters)
  {
    clusters.clear();

    if (!initCompute() || input_->empty() || indices_->empty())
      return;

    if (!tree_) {
      if (input_->isOrganized())
        tree_.reset(new pcl::search::OrganizedNeighbor<PointT>);
      else
        tree_.reset(new pcl::search::KdTree<PointT>);
    }
    tree_->setInputCloud(input_, indices_);

    std::vector<size_t> labels(input_->size(), -1);
    std::vector<bool> removed(input_->size(), false);

    pcl::Indices nn_indices;
    std::vector<float> nn_distances;
    auto nn_distance_threshold = std::pow((1.0 - quality_) * cluster_tolerance_, 2.0);

    Graph g;
    std::queue<size_t> queue;

    {
      size_t label = 0;
      for (auto index : *indices_) {
        if (removed.at(index))
          continue;

        boost::add_edge(label, label, g);

        queue.push(index);
        while (!queue.empty()) {
          auto p = queue.front();
          queue.pop();
          if (removed.at(p)) {
            continue;
          }

          tree_->radiusSearch(p, cluster_tolerance_, nn_indices, nn_distances);

          for (std::size_t i = 0; i < nn_indices.size(); ++i) {
            auto q = nn_indices.at(i);
            auto q_label = labels.at(q);

            if (q_label != -1 && q_label != label) {
              boost::add_edge(label, q_label, g);
            }

            if (removed.at(q)) {
              continue;
            }

            labels.at(q) = label;

            // Must be <= to remove self (p).
            if (nn_distances.at(i) <= nn_distance_threshold) {
              removed.at(q) = true;
            }
            else {
              queue.push(q);
            }
          }
        }

        label++;
      }
    }

    // Merge labels.

    std::vector<size_t> label_map(boost::num_vertices(g));
    auto num_components = boost::connected_components(g, label_map.data());
    clusters.resize(num_components);

    for (auto index : *indices_) {
      auto label = labels.at(index);
      auto new_label = label_map.at(label);
      clusters.at(new_label).indices.push_back(index);
    }

    // Remove small and large clusters.

    auto read = clusters.begin();
    auto write = clusters.begin();
    for (; read != clusters.end(); ++read) {
      if (read->indices.size() >= min_cluster_size_ &&
          read->indices.size() <= max_cluster_size_) {
        if (read != write) {
          *write = std::move(*read);
        }
        ++write;
      }
    }
    clusters.resize(std::distance(clusters.begin(), write));

    deinitCompute();
  }

private:
  double cluster_tolerance_;
  unsigned int max_cluster_size_;
  unsigned int min_cluster_size_;
  double quality_;
  KdTreePtr tree_;
};

#endif  // CATKIN_WS_FAST_EUCLIDEAN_CLUSTERING_H
