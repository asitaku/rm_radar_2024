//
// Created by jg on 2022/4/2.
//

#ifndef SRC_RM_RADAR_INCLUDE_PREALARM_H_
#define SRC_RM_RADAR_INCLUDE_PREALARM_H_
#include "lidar.h"
#include <image_transport/image_transport.h>
#include <std_msgs/Int8MultiArray.h>
#include <ros/time.h>
#include <rm_msgs/ClientMapDataArray.h>

namespace rm_radar
{
#define DEPTH_OMISSION 5
#define AREA_NUM 5

class UI : public nodelet::Nodelet
{
public:
  UI();
  ~UI() override;

  void onInit() override;

private:
  ros::NodeHandle nh_;
  cv::Mat map_;
  image_transport::Publisher img_pub_;
  ros::Publisher warning_pub_;

  ros::Subscriber target_sub_;

  void targetCB(const rm_msgs::ClientMapDataArray& target_msg);

  float fx_{}, fy_{};

  std::vector<boost::shared_ptr<std::vector<cv::Point>>> prealarm_areas_{};

  void getParm();

  bool target_is_red_{}, is_polys_{};

  int check(const cv::Point& target, const int& robot_id,
            const std::vector<boost::shared_ptr<std::vector<cv::Point>>>& prealarm_areas);
};
}  // namespace rm_radar
#endif  // SRC_RM_RADAR_INCLUDE_PREALARM_H_
