//
// Created by ywj on 24-8-1.
//

#ifndef CATKIN_WS_TIME_CHANGE_MISSING_HPP
#define CATKIN_WS_TIME_CHANGE_MISSING_HPP

#include <iostream>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace rm_tower
{
class TimeChangeMissing {
public:
  explicit TimeChangeMissing(XmlRpc::XmlRpcValue& missing_array, bool target_is_red) : current_index_(0), last_detected_time_(ros::Time::now())
  {
    current_num_ = static_cast<int>(missing_array.size());
    if (!target_is_red)
      for (int i = 0; i < current_num_; ++i)
        ori_missing_array_.emplace_back(double(missing_array[i][0]), double(missing_array[i][1]));
    else
      for (int i = 0; i < current_num_; ++i)
        ori_missing_array_.emplace_back(28.0 - double(missing_array[i][0]), 15.0 - double(missing_array[i][1]));
    current_missing_array_.assign(ori_missing_array_.begin(), ori_missing_array_.end());
  }

  void setBaseAttackedArray(XmlRpc::XmlRpcValue& array, bool target_is_red)
  {
    if (!target_is_red)
      for (int i = 0; i < static_cast<int>(array.size()); ++i)
        base_attacked_missing_array_.emplace_back(double(array[i][0]), double(array[i][1]));
    else
      for (int i = 0; i < static_cast<int>(array.size()); ++i)
        base_attacked_missing_array_.emplace_back(28.0 - double(array[i][0]), 15.0 - double(array[i][1]));
  }

  void setHeroAttackedArray(XmlRpc::XmlRpcValue& array, bool target_is_red)
  {
    if (!target_is_red)
      for (int i = 0; i < static_cast<int>(array.size()); ++i)
        hero_r3_attacked_missing_array_.emplace_back(double(array[i][0]), double(array[i][1]));
    else
      for (int i = 0; i < static_cast<int>(array.size()); ++i)
        hero_r3_attacked_missing_array_.emplace_back(28.0 - double(array[i][0]), 15.0 - double(array[i][1]));
  }

  cv::Point2f getCurrentMissing()
  {
    return current_missing_array_[current_index_];
  }

  void updateByDetected(cv::Point2f& point)
  {
    if (current_missing_array_.size() <= 1)
      return;
    float min = sqrt((point.x - current_missing_array_[0].x) * (point.x - current_missing_array_[0].x) +
                     (point.y - current_missing_array_[0].y) * (point.y - current_missing_array_[0].y));
    int min_index = 0;
    for (int i = 1; i < current_num_; i++)
    {
      float temp = sqrt((point.x - current_missing_array_[i].x) * (point.x - current_missing_array_[i].x) +
                        (point.y - current_missing_array_[i].y) * (point.y - current_missing_array_[i].y));
      if (min > temp)
      {
        min_index = i;
        min = temp;
      }
    }
    current_index_ = min_index;
  }

  void update()
  {
    if (ros::Time::now() - last_detected_time_ > ros::Duration(5.0))
    {
      current_index_ = ++current_index_ % current_num_;
      last_detected_time_ = ros::Time::now();
    }
  }

  void updateBaseAttackedArray()
  {
    if (base_attacked_missing_array_.empty())
      return;
    current_index_ = 0;
    current_num_ = base_attacked_missing_array_.size();
    current_missing_array_.clear();
    current_missing_array_.assign(base_attacked_missing_array_.begin(), base_attacked_missing_array_.end());
  }

  void updateHeroAttackedArray()
  {
    if (hero_r3_attacked_missing_array_.empty())
      return;
    current_index_ = 0;
    current_num_ = hero_r3_attacked_missing_array_.size();
    current_missing_array_.clear();
    current_missing_array_.assign(hero_r3_attacked_missing_array_.begin(), hero_r3_attacked_missing_array_.end());
  }

  void addMissingPoint(const cv::Point2f& new_missing_point)
  {
    current_missing_array_.emplace_back(new_missing_point);
    current_num_++;
  }

  void disableMissPoint(int index)
  {
    auto i = current_missing_array_.begin() + index;
      current_missing_array_.erase(i);
  }

  void reset()
  {
    last_detected_time_ = ros::Time::now();
  }

  void resetArray()
  {
    current_index_ = 0;
    current_num_ = ori_missing_array_.size();
    current_missing_array_.clear();
    current_missing_array_.assign(ori_missing_array_.begin(), ori_missing_array_.end());
  }

  void pop_back()
  {
    ori_missing_array_.pop_back();
    resetArray();
  }

private:
  ros::Time last_detected_time_;
  std::vector<cv::Point2f> current_missing_array_;
  std::vector<cv::Point2f> ori_missing_array_;
  std::vector<cv::Point2f> base_attacked_missing_array_;
  std::vector<cv::Point2f> hero_r3_attacked_missing_array_;
  int current_index_;
  int current_num_ = 0;
};
}

#endif // CATKIN_WS_TIME_CHANGE_MISSING_HPP
