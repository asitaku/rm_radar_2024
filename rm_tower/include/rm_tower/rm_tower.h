//
// Created by ywj on 24-5-1.
//

#ifndef RM_TOWER_H
#define RM_TOWER_H
#include "rm_msgs/ClientMapReceiveData.h"
#include "rm_msgs/DartPositionInCam.h"
#include "rm_msgs/DartRemainingTime.h"
#include "rm_msgs/EventData.h"
#include "rm_msgs/GameRobotHp.h"
#include "rm_msgs/RadarInfo.h"
#include "rm_msgs/RadarToSentry.h"
#include "rm_msgs/RadarWarningState.h"
#include "rm_msgs/SentryAttackingTarget.h"
#include "time_change_missing.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mutex>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <pluginlib/class_list_macros.h>
#include <rm_common/filters/filters.h>
#include <rm_msgs/ClientMapData.h>
#include <rm_msgs/ClientMapDataArray.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/RadarInfo.h>
#include <rm_msgs/RadarMarkData.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <thread>
#include "time_change_missing.hpp"

#define ROBOT_NUM 6

const std::vector<std::string> robot_name{"Hero", "Engineer", "Standard3", "Standard4", "Standard5", "Sentry"};

template <typename T>
class Vector2WithFilter
{
public:
  explicit Vector2WithFilter(int num_data)
  {
    for (int i = 0; i < 2; i++)
      filter_vector_.push_back(std::make_shared<MovingAverageFilter<T>>(num_data));
  }
  void input(T vector[2])
  {
    for (int i = 0; i < 2; i++)
      filter_vector_[i]->input(vector[i]);
  }
  void clear()
  {
    for (int i = 0; i < 2; i++)
      filter_vector_[i]->clear();
  }
  T x()
  {
    return filter_vector_[0]->output();
  }
  T y()
  {
    return filter_vector_[1]->output();
  }

private:
  std::vector<std::shared_ptr<MovingAverageFilter<T>>> filter_vector_;
};

struct TrackPrediction
{
  float length = 0.0f;
  double direction = 0.0f;
  double wall = 0.0f;
};

namespace rm_tower
{
void onMouse(int event, int x, int y, int flags, void* param);

class RmTower : public nodelet::Nodelet
{
public:
  RmTower();
  ~RmTower() override;
  void onInit() override;
//    if (dart_missing_time_ != 0)
//      ROS_INFO("dart_warning!");
  ros::NodeHandle nh_;
  std::mutex tower_mtx_;
  bool get_rect_;
  cv::Rect dart_rect_;
  bool dart_get_;

private:
  void gameStatusCallBack(const rm_msgs::GameStatus& game_status);
  void radarMarkCallBack(const rm_msgs::RadarMarkData& radar_mark);
  void radarInfoCallBack(const rm_msgs::RadarInfo& radar_info);

  void radarLeftMapCallBack(const rm_msgs::ClientMapDataArray& data_array);
  void radarRightMapCallBack(const rm_msgs::ClientMapDataArray& data_array);
  void radarMapCallBack(const rm_msgs::ClientMapDataArray& data_array);

  void gameRobotHpCallBack(const rm_msgs::GameRobotHp& robotHp);
  void eventDataCallBack(const rm_msgs::EventData& eventData);
  void dartRemainingTimeDataCallBack(const rm_msgs::DartRemainingTime& dartRemainingTime);
  void sentryAttackingTargetCallBack(const rm_msgs::SentryAttackingTarget& sentryAttackingTarget);
  void rightCameraCallBack(const sensor_msgs::CompressedImageConstPtr& image);
  void getDartRect();
  void mapMain();
  void getWall(cv::Mat& image);
  bool collision_detection(const rm_msgs::ClientMapData& last_position, const rm_msgs::ClientMapData& current_position);

  void initialize(ros::NodeHandle& nh);

  boost::shared_mutex mark_lock_;
  std::mutex map_mtx_;
  bool target_is_red_;
  int dart_target_;
  cv::Mat map_binary_;
  rm_msgs::ClientMapDataArray marked_target_to_sentry_;
  bool engineer_marked_;
  bool dart_initialized_ = false;
  cv::Mat image_;
  cv::Mat warning_image_;
  int dart_time_ = 0;

  ros::Subscriber game_status_sub_;
  ros::Subscriber radar_mark_sub_;
  ros::Subscriber radar_info_sub_;
  ros::Subscriber dart_remaining_time_data_sub_;
  ros::Subscriber sentry_to_radar_sub_;

  ros::Subscriber radar_left_map_sub_;
  ros::Subscriber radar_right_map_sub_;
  ros::Subscriber radar_map_sub_;

  ros::Subscriber dart_position_sub_;
  ros::Subscriber right_camera_sub_;

  ros::Subscriber game_robot_hp_sub_;
  ros::Subscriber event_data_sub_;

  ros::Publisher radar_receive_pub_;
  ros::Publisher radar_test_pub_;
  ros::Publisher radar_cmd_pub_;
  ros::Publisher radar_to_referee_pub_;
  ros::Publisher radar_custom_info_pub_;
  ros::Publisher radar_warning_pub_;

  ros::Publisher dart_img_pub_;

  rm_msgs::RadarMarkData mark_data_{};
  std::vector<int> mark_loss_;

  std::vector<int> mark_queue_;
  int target_id_;

  std::vector<int> radar_info_;
  std::vector<int> radar_time_;
  std::vector<bool> map_todo_;
  std::vector<int> mark_info_;
  std::vector<int> pub_list_;
  std::vector<int> temp_;
  std::vector<rm_msgs::ClientMapData> map_radar_;
  bool todo_double_;
  bool sentry_get_;
  int dart_missing_time_;

  std::vector<TimeChangeMissing> missing_selector_;

//  std::vector<std::shared_ptr<Vector3WithFilter<float>>> vel_buffer_;
  std::vector<cv::Point2f> vel_;
//  std::vector<rm_msgs::ClientMapReceiveData> map_list_;
  rm_msgs::ClientMapDataArray left_map_list_;
  rm_msgs::ClientMapDataArray right_map_list_;
  rm_msgs::ClientMapDataArray map_pub_list_;
  rm_msgs::ClientMapDataArray map_last_pub_list_;
  std::vector<bool> left_detected_list_;
  std::vector<bool> right_detected_list_;
  std::vector<bool> current_detected_;
  ros::Time last_pub_time_;
  std::vector<bool> initialize_;
  std::vector<bool> last_detected_;
  std::vector<int> missing_time_;
  std::vector<int> max_missing_time_;
  rm_msgs::ClientMapDataArray missing_time_array_;
//  rm_msgs::RadarWarningState radar_warning_state_;
  ros::Time last_warning_dart_time_;
  ros::Time last_warning_flying_slope_time_;
  ros::Time last_ready_double_time_;

  std::vector<ros::Time> last_marked_time_;

  std::vector<std::vector<int>> wall_;

  int our_base_hp_ = 5000;
  bool ring_elevated_ground_occupied_ = false;
  std::vector<int> robot_hp_;

//  std::vector<std::shared_ptr<Vector2WithFilter<float>>> vel_buffer_;
//  std::vector<std::shared_ptr<Vector2WithFilter<float>>> position_buffer_;

  rm_msgs::ClientMapData sentry_target_to_radar_msg_;
  ros::Time last_base_attacked_time_;
  ros::Time last_hero_attacked_time_;

  bool base_attacked_ = false;
  bool hero_on_r3_ = false;
  bool hero_attacked_ = false;

  int radar_cmd_;
  int game_process_;
  rm_msgs::GameRobotHp last_robot_hp_;
};
}  // namespace rm_tower

#endif  // RM_TOWER_H
