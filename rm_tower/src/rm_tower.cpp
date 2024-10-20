//
// Created by ywj on 24-5-1.
//

#include "rm_tower/rm_tower.h"
namespace rm_tower
{
void RmTower::onInit()
{
  nh_ = this->getMTPrivateNodeHandle();

  nh_.getParam("target_is_red", target_is_red_);

  cv::Mat image = cv::imread(ros::package::getPath("rm_tower") + "/config/radar_thresh_1.jpg");
  getWall(image);
  warning_image_ = cv::imread(ros::package::getPath("rm_tower") + "/config/map_warning.jpg");
//  cv::threshold(warning_image_, warning_image_, 100, 255, cv::THRESH_BINARY);
//  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
//  cv::erode(warning_image_, warning_image_, element);

  game_status_sub_ = nh_.subscribe("/rm_referee/game_status", 1, &RmTower::gameStatusCallBack, this);
  radar_mark_sub_ = nh_.subscribe("/rm_referee/radar_mark", 1, &RmTower::radarMarkCallBack, this);
  radar_info_sub_ = nh_.subscribe("/rm_referee/radar_info", 1, &RmTower::radarInfoCallBack, this);

  sentry_to_radar_sub_ = nh_.subscribe("/rm_referee/sentry_target_to_radar", 1, &RmTower::sentryAttackingTargetCallBack, this);

  radar_left_map_sub_ = nh_.subscribe("/rm_radar_left/detections", 5, &RmTower::radarLeftMapCallBack, this);
  radar_right_map_sub_ = nh_.subscribe("/rm_radar_right/detections", 5, &RmTower::radarRightMapCallBack, this);

  right_camera_sub_ = nh_.subscribe("/hk_camera_right/image_raw/compressed", 1, &RmTower::rightCameraCallBack, this);

//  radar_map_sub_ = nh_.subscribe("/rm_radar/detections", 5, &RmTower::radarMapCallBack, this);

  game_robot_hp_sub_ = nh_.subscribe("/rm_referee/game_robot_hp", 1, &RmTower::gameRobotHpCallBack, this);
  event_data_sub_ = nh_.subscribe("/rm_referee/event_data", 1, &RmTower::eventDataCallBack, this);
  dart_remaining_time_data_sub_ = nh_.subscribe("/rm_referee/dart_remaining_time_data", 1, &RmTower::dartRemainingTimeDataCallBack, this);

  dart_img_pub_ = nh_.advertise<sensor_msgs::Image>("/dart_test", 1);
  radar_test_pub_ = nh_.advertise<rm_msgs::ClientMapDataArray>("/rm_radar_test", 10);
  radar_receive_pub_ = nh_.advertise<rm_msgs::ClientMapReceiveData>("/rm_radar", 10);
  radar_cmd_pub_ = nh_.advertise<rm_msgs::RadarInfo>("/radar_cmd", 1);
  radar_to_referee_pub_ = nh_.advertise<rm_msgs::RadarToSentry>("/radar_to_referee", 20);
  radar_custom_info_pub_ = nh_.advertise<std_msgs::String>("/custom_info", 5);
//  radar_warning_pub_ = nh_.advertise<rm_msgs::RadarWarningState>("/radar_warning", 1);

  initialize(nh_);

  ROS_INFO("RmTower Start Work!");

  std::thread main_thread(&RmTower::mapMain, this);
  std::thread dart_thread(&RmTower::getDartRect, this);

  main_thread.join();
  dart_thread.join();
}

RmTower::RmTower()
{
  mark_loss_.assign(ROBOT_NUM, 0);
  radar_info_.assign(3, 0);
  vel_.assign(ROBOT_NUM, cv::Point2f(0.0f, 0.0f));
  map_last_pub_list_.array.assign(ROBOT_NUM, rm_msgs::ClientMapData());
  map_pub_list_.array.assign(ROBOT_NUM, rm_msgs::ClientMapData());
  left_map_list_.array.assign(ROBOT_NUM, rm_msgs::ClientMapData());
  right_map_list_.array.assign(ROBOT_NUM, rm_msgs::ClientMapData());
  target_is_red_ = true;
  radar_cmd_ = 0;
//  engineer_in_land_ = true;
  todo_double_ = false;
  mark_info_.assign(ROBOT_NUM, 0);
  last_pub_time_ = ros::Time::now();
  sentry_get_ = false;
  left_detected_list_.assign(ROBOT_NUM, false);
  right_detected_list_.assign(ROBOT_NUM, false);
  max_missing_time_ = std::vector<int>{5, 5, 5, 5, 5, 10};
  missing_time_.assign(ROBOT_NUM, 0);

//  vel_buffer_.assign(ROBOT_NUM, std::make_shared<Vector2WithFilter<float>>(5));
//  position_buffer_.assign(ROBOT_NUM, std::make_shared<Vector2WithFilter<float>>(2));

  current_detected_.assign(ROBOT_NUM, false);
  engineer_marked_ = false;

  dart_get_ = false;
//  radar_warning_state_.warning_dart = false;
//  radar_warning_state_.warning_flying_slope = false;
//  radar_warning_state_.enable_radar_double = false;

  robot_hp_.assign(ROBOT_NUM, 100);
  last_warning_dart_time_ = ros::Time::now();
  last_warning_flying_slope_time_ = ros::Time::now();
  last_ready_double_time_ = ros::Time::now();

  last_marked_time_.assign(ROBOT_NUM, ros::Time::now());
  dart_target_ = 0;
}

RmTower::~RmTower() = default;

void RmTower::gameStatusCallBack(const rm_msgs::GameStatus& game_status)
{
  game_process_ = game_status.game_progress;
  if (game_process_ == 4) {
    if (todo_double_)
      if (game_status.stage_remain_time <= 60) {
        //        if ((game_status.stage_remain_time >= 45 && radar_info_[1] == 1) ||
        //            game_status.stage_remain_time < 45) {
        rm_msgs::RadarInfo radar_info;
        radar_cmd_++;
        radar_info.radar_info = radar_cmd_;
        radar_cmd_pub_.publish(radar_info);
        todo_double_ = false;
        std_msgs::String msg;
        msg.data = "RadarAutoDouble";
        radar_custom_info_pub_.publish(msg);
        //        }
      }
  }
  else
  {
    todo_double_ = false;
    radar_cmd_ = 0;
  }
}

void RmTower::radarMarkCallBack(const rm_msgs::RadarMarkData& radar_mark)
{
  mark_info_[0] = radar_mark.mark_hero_progress;
  mark_info_[1] = radar_mark.mark_engineer_progress;
  if (radar_mark.mark_engineer_progress >= 100 && map_last_pub_list_.array[1].target_position_x > 12.28362f && map_last_pub_list_.array[1].target_position_x < 15.75238f && map_last_pub_list_.array[1].target_position_y > 5.31048f && map_last_pub_list_.array[1].target_position_y < 9.72552f) {
      engineer_marked_ = true;
  } else
    engineer_marked_ = false;
  mark_info_[2] = radar_mark.mark_standard_3_progress;
  mark_info_[3] = radar_mark.mark_standard_4_progress;
  mark_info_[4] = radar_mark.mark_standard_5_progress;
  mark_info_[5] = radar_mark.mark_sentry_progress;
}

void RmTower::radarInfoCallBack(const rm_msgs::RadarInfo& radar_info)
{
  if (game_process_ == 4) {
    int temp = radar_info.radar_info;
    for (int i = 0; i < 3; ++i) {
      radar_info_[i] = temp % 2;
      temp /= 2;
    }

    if (radar_info_[0] + radar_info_[1] >= 1) {
      if (radar_info_[2] == 0) {
        todo_double_ = true;
        if (ros::Time::now() - last_ready_double_time_ > ros::Duration(8.0)) {
          std_msgs::String msg;
          msg.data = "Ready_Double";
          radar_custom_info_pub_.publish(msg);
          last_ready_double_time_ = ros::Time::now();
        }
        //      radar_warning_state_.enable_radar_double = true;
      }
      //    else
      //      radar_warning_state_.enable_radar_double = false;
    }
  }
}

void RmTower::gameRobotHpCallBack(const rm_msgs::GameRobotHp& robotHp)
{
  if (game_process_ == 4) {
    int our_hero_dhp = 0;
    int our_base_dhp = 0;
    if (target_is_red_)
    {
      our_base_dhp = last_robot_hp_.blue_base_hp - robotHp.blue_base_hp;
      our_hero_dhp = last_robot_hp_.blue_1_robot_hp - robotHp.blue_1_robot_hp;
    }
    else
    {
      our_base_dhp = last_robot_hp_.red_base_hp - robotHp.red_base_hp;
      our_hero_dhp = last_robot_hp_.red_1_robot_hp - robotHp.red_1_robot_hp;
    }

    if (our_hero_dhp > 0 && hero_on_r3_) {
      if (!hero_attacked_) {
        for (auto i : missing_selector_)
          i.updateHeroAttackedArray();
        last_hero_attacked_time_ = ros::Time::now();
        hero_attacked_ = true;
      } else
        last_hero_attacked_time_ = ros::Time::now();
    } else if (our_hero_dhp == 0) {
      if (hero_attacked_) {
        if (ros::Time::now() - last_hero_attacked_time_ > ros::Duration(12.0)) {
          for (auto i : missing_selector_)
            i.resetArray();
          hero_attacked_ = false;
        }
      }
    }

    if (our_base_dhp > 0 && our_base_dhp < 1000 && our_base_dhp != 500 &&
        !ring_elevated_ground_occupied_) {
      if (!base_attacked_) {
        for (auto i : missing_selector_)
          i.updateBaseAttackedArray();
        last_base_attacked_time_ = ros::Time::now();
        base_attacked_ = true;
      } else
        last_base_attacked_time_ = ros::Time::now();
    } else if (our_base_dhp == 0) {
      if (base_attacked_) {
        if (ros::Time::now() - last_base_attacked_time_ > ros::Duration(12.0)) {
          for (auto i : missing_selector_)
            i.resetArray();
          base_attacked_ = false;
        }
      }
    }
  }
}

void RmTower::eventDataCallBack(const rm_msgs::EventData& eventData)
{
  if (game_process_ == 4) {
    if (eventData.ring_elevated_ground_state == 2)
      ring_elevated_ground_occupied_ = true;
    else
      ring_elevated_ground_occupied_ = false;

    if (eventData.r3_state != 0)
      hero_on_r3_ = true;
    else
      hero_on_r3_ = false;
  }
}

void RmTower::radarLeftMapCallBack(const rm_msgs::ClientMapDataArray& data_array)
{
  for (auto i : data_array.array)
  {
    left_detected_list_[i.target_robot_ID] = true;
    left_map_list_.array[i.target_robot_ID] = i;
  }
}

void RmTower::dartRemainingTimeDataCallBack(const rm_msgs::DartRemainingTime &dartRemainingTime)
{
  if (game_process_ == 4) {
    int dart_target = dartRemainingTime.dart_aim_state / 32;
    if (dart_target != dart_target_ && todo_double_) {
      dart_target_ = dart_target;
      rm_msgs::RadarInfo radar_info;
      radar_cmd_++;
      ROS_INFO("%d", radar_cmd_);
      radar_info.radar_info = radar_cmd_;
      radar_cmd_pub_.publish(radar_info);
      std_msgs::String msg;
      msg.data = "TriggerDouble";
      radar_custom_info_pub_.publish(msg);
      todo_double_ = false;
    }
  }
}

void RmTower::getWall(cv::Mat &image)
{
  cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
  cv::threshold(image, image, 200, 255, cv::THRESH_BINARY);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
  cv::erode(image, image, element);

  map_binary_ = image.clone();
}

void RmTower::sentryAttackingTargetCallBack(const rm_msgs::SentryAttackingTarget &sentryAttackingTarget)
{
  if (sentryAttackingTarget.target_robot_ID == 6)
    return;
  rm_msgs::ClientMapData msg;
  msg.target_robot_ID = sentryAttackingTarget.target_robot_ID - 1;
  if (msg.target_robot_ID == 6)
    msg.target_robot_ID = 5;
  msg.target_position_x = sentryAttackingTarget.target_position_x - 0.2f;
  msg.target_position_y = sentryAttackingTarget.target_position_y - 0.5f;
  sentry_target_to_radar_msg_ = msg;
  sentry_get_ = true;
}

void RmTower::radarRightMapCallBack(
    const rm_msgs::ClientMapDataArray &data_array)
{
  for (auto i : data_array.array)
  {
    right_detected_list_[i.target_robot_ID] = true;
    right_map_list_.array[i.target_robot_ID] = i;
  }
}

void RmTower::radarMapCallBack(
    const rm_msgs::ClientMapDataArray &data_array)
{
  for (auto i : data_array.array)
  {
    ROS_INFO("%d", i.target_robot_ID);
    current_detected_[i.target_robot_ID] = true;
//    map_pub_list_.array[i.target_robot_ID] = i;
    float position[2];
    position[0] = i.target_position_x;
    position[1] = i.target_position_y;
//    position_buffer_[i.target_robot_ID]->input(position);
  }
}

bool RmTower::collision_detection(
    const rm_msgs::ClientMapData &last_position,
    const rm_msgs::ClientMapData &current_position) {
  float last_x = (last_position.target_position_x + 1.5f) / 0.029f;
  float last_y = (last_position.target_position_y + 1.5f) / 0.029f;

  float current_x = (current_position.target_position_x + 1.5f) / 0.029f;
  float current_y = (current_position.target_position_y + 1.5f) / 0.029f;

  float k = (current_y - last_y) / (current_x - last_x);
  float b = current_y - k * current_x;

  int current_large_last = 1;
  int init_x = int(last_x);
  int target_x = int(current_x);
  if (last_x > current_x)
    current_large_last = -1;
  int step = 3 * current_large_last;

  for (int i = init_x + current_large_last; i * current_large_last <= target_x * current_large_last; i += step)
    if (map_binary_.at<uchar>(i, int(k * i + b)) == 0) {
      return true;
    }
  return false;
}

void RmTower::rightCameraCallBack(
    const sensor_msgs::CompressedImageConstPtr &image)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);
  cv_image->image.copyTo(image_);
  if (dart_get_)
  {
    cv::Mat dart_img = image_(dart_rect_);
    cv::Mat img_hsv;
    cv::cvtColor(dart_img, img_hsv, cv::COLOR_BGR2HSV);

    cv::inRange(img_hsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), img_hsv);
//    cv::threshold(img_hsv, img_hsv, 10, 255, cv::THRESH_BINARY);
    dart_missing_time_++;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_hsv, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    unsigned long contours_size = contours.size();
    for (size_t i = 0; i < contours_size; i++)
    {
      if (cv::contourArea(contours[i]) > 100)
      {
        cv::drawContours(dart_img, contours, i, cv::Scalar(0, 0, 255), 4);
        dart_missing_time_ = 0;
      }
    }
    if (dart_missing_time_ != 0)
    {
      if (ros::Time::now() - last_warning_dart_time_ > ros::Duration(45.0f))
      {
        std_msgs::String msg;
        msg.data = "Warning_Dart";
        for (int i = 0; i < 5; i++)
          radar_custom_info_pub_.publish(msg);
        last_warning_dart_time_ = ros::Time::now();
      }
    }
//      radar_warning_state_.warning_dart = true;
//    else
//      radar_warning_state_.warning_dart = false;

    dart_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", dart_img).toImageMsg());
  }
//  cv::rectangle(cv_image_->image, dart_rect_, cv::Scalar(255, 255, 0), 4);
}

void RmTower::getDartRect()
{
  cv::namedWindow("dart_warn", cv::WINDOW_KEEPRATIO);
  cv::setMouseCallback("dart_warn", rm_tower::onMouse, (void*)this);
  dart_get_ = false;
  get_rect_ = false;
  while (!dart_get_)
  {
    if (image_.empty())
      continue;

    if (get_rect_)
      cv::rectangle(image_, dart_rect_.tl(), dart_rect_.br(), cv::Scalar(255, 255, 0), 4);
    cv::imshow("dart_warn", image_);
    cv::waitKey(10);
  }
  cv::destroyWindow("dart_warn");
//  right_camera_sub_.shutdown();
}

void RmTower::mapMain()
{
  while (true)
  {
    if (ros::Time::now() - last_pub_time_ > ros::Duration(0.2))
    {
      for (int i = 0; i < ROBOT_NUM; i++)
      {
        missing_selector_[i].update();

        if (mark_info_[i] >= 100)
          missing_selector_[i].reset();

        if (robot_hp_[i] == 0)
        {
          vel_[i].x = 0;
          vel_[i].y = 0;
        }
        if (vel_[i].x <= 0.05f)
          vel_[i].x = 0;
        if (vel_[i].y <= 0.05f)
          vel_[i].y = 0;

        if (left_detected_list_[i])
        {
          map_pub_list_.array[i] = left_map_list_.array[i];
          current_detected_[i] = true;
        }
        if (right_detected_list_[i])
        {
          map_pub_list_.array[i] = right_map_list_.array[i];
          current_detected_[i] = true;
        }
        if (sentry_get_) {
          map_pub_list_.array[sentry_target_to_radar_msg_.target_robot_ID] = sentry_target_to_radar_msg_;
          sentry_get_ = false;
          current_detected_[i] = true;
        }

        if (current_detected_[i])
        {
          if (missing_time_[i] > 0)
          {
            vel_[i].x = 0;
            vel_[i].y = 0;
          }
          else
          {
            if (collision_detection(map_last_pub_list_.array[i],
                                    map_pub_list_.array[i]))
            {
              float radius =
                  sqrt(vel_[i].x * vel_[i].x + vel_[i].y * vel_[i].y);
              double last_direction = atan(vel_[i].y / vel_[i].x);
              double step = CV_PI / 4;
              double left_angle = last_direction + step;
              double right_angle = last_direction - step;
              rm_msgs::ClientMapData left_position =
                  map_last_pub_list_.array[i];
              rm_msgs::ClientMapData right_position =
                  map_last_pub_list_.array[i];
              while (left_angle - right_angle < CV_PI) {
                left_position.target_position_x =
                    map_last_pub_list_.array[i].target_position_x +
                    radius * cos(left_angle);
                left_position.target_position_y =
                    map_last_pub_list_.array[i].target_position_y +
                    radius * sin(left_angle);
                if (!collision_detection(map_last_pub_list_.array[i],
                                         left_position)) {
                  map_pub_list_.array[i] = left_position;
                  vel_[i].x = radius * cos(left_angle);
                  vel_[i].y = radius * sin(left_angle);
                  break;
                }
                left_angle += step;
                right_position.target_position_x =
                    map_last_pub_list_.array[i].target_position_x +
                    radius * cos(right_angle);
                right_position.target_position_y =
                    map_last_pub_list_.array[i].target_position_y +
                    radius * sin(right_angle);
                if (!collision_detection(map_last_pub_list_.array[i],
                                         right_position)) {
                  map_pub_list_.array[i] = right_position;
                  vel_[i].x = radius * cos(right_angle);
                  vel_[i].y = radius * sin(right_angle);
                  break;
                }
                right_angle -= step;
              }
              if (left_angle - right_angle >= CV_PI)
              {
                map_pub_list_.array[i] = map_last_pub_list_.array[i];
                vel_[i].x = 0;
                vel_[i].y = 0;
              }
            } else {
              vel_[i].x = map_pub_list_.array[i].target_position_x -
                          map_last_pub_list_.array[i].target_position_x;
              vel_[i].y = map_pub_list_.array[i].target_position_y -
                          map_last_pub_list_.array[i].target_position_y;
            }
          }
          missing_time_[i] = 0;
          last_marked_time_[i] = ros::Time::now();

        }

        if (!current_detected_[i])
        {
          missing_time_[i]++;
          if (mark_info_[i] >= 100)
          {
            map_pub_list_.array[i].target_position_x = map_last_pub_list_.array[i].target_position_x + vel_[i].x;
            map_pub_list_.array[i].target_position_y = map_last_pub_list_.array[i].target_position_y + vel_[i].y;
            if (collision_detection(map_last_pub_list_.array[i], map_pub_list_.array[i]))
            {
              if (missing_time_[i] < max_missing_time_[i] / 4)
              {
                float radius =
                    sqrt(vel_[i].x * vel_[i].x + vel_[i].y * vel_[i].y);
                double last_direction = atan(vel_[i].y / vel_[i].x);
                double step = CV_PI / 4;
                double left_angle = last_direction + step;
                double right_angle = last_direction - step;
                rm_msgs::ClientMapData left_position =
                    map_last_pub_list_.array[i];
                rm_msgs::ClientMapData right_position =
                    map_last_pub_list_.array[i];
                while (left_angle - right_angle < CV_PI) {
                  left_position.target_position_x =
                      map_last_pub_list_.array[i].target_position_x +
                      radius * cos(left_angle);
                  left_position.target_position_y =
                      map_last_pub_list_.array[i].target_position_y +
                      radius * sin(left_angle);
                  if (!collision_detection(map_last_pub_list_.array[i],
                                           left_position)) {
                    map_pub_list_.array[i] = left_position;
                    vel_[i].x = radius * cos(left_angle);
                    vel_[i].y = radius * sin(left_angle);
                    break;
                  }
                  left_angle += step;
                  right_position.target_position_x =
                      map_last_pub_list_.array[i].target_position_x +
                      radius * cos(right_angle);
                  right_position.target_position_y =
                      map_last_pub_list_.array[i].target_position_y +
                      radius * sin(right_angle);
                  if (!collision_detection(map_last_pub_list_.array[i],
                                           right_position)) {
                    map_pub_list_.array[i] = right_position;
                    vel_[i].x = radius * cos(right_angle);
                    vel_[i].y = radius * sin(right_angle);
                    break;
                  }
                  right_angle -= step;
                }
                if (left_angle - right_angle >= CV_PI)
                {
                  map_pub_list_.array[i] = map_last_pub_list_.array[i];
                  vel_[i].x = 0;
                  vel_[i].y = 0;
                }
              }
              else
              {
                map_pub_list_.array[i] = map_last_pub_list_.array[i];
                vel_[i].x = 0;
                vel_[i].y = 0;
              }
            }
            last_marked_time_[i] = ros::Time::now();
          }
          else
          {
            vel_[i].x = 0;
            vel_[i].y = 0;
            if (ros::Time::now() - last_marked_time_[i] <= ros::Duration(3.0f))
            {
              map_pub_list_.array[i].target_robot_ID = i;
              map_pub_list_.array[i].target_position_x = 0;
              map_pub_list_.array[i].target_position_y = 0;
            }
            else {
              map_pub_list_.array[i].target_robot_ID = i;
              map_pub_list_.array[i].target_position_x =
                  missing_selector_[i].getCurrentMissing().x;
              map_pub_list_.array[i].target_position_y =
                  missing_selector_[i].getCurrentMissing().y;
            }
          }
        }

        if (mark_info_[i] > 100)
        {
          rm_msgs::RadarToSentry msg;
          msg.robot_ID = map_pub_list_.array[i].target_robot_ID + 1;
          if (msg.robot_ID == 6)
            msg.robot_ID = 7;
          msg.engineer_marked = engineer_marked_;
          msg.position_x = map_pub_list_.array[i].target_position_x;
          msg.position_y = map_pub_list_.array[i].target_position_y;
          radar_to_referee_pub_.publish(msg);
        }

        float max_vel = 0.8f;
        float vel = sqrt((vel_[i].x * vel_[i].x + vel_[i].y * vel_[i].y));
        if (vel > max_vel)
        {
          float angle = vel_[i].y / vel_[i].x;
          vel_[i].x = vel_[i].x / abs(vel_[i].x) * max_vel * cos(angle);
          vel_[i].y = vel_[i].y / abs(vel_[i].y) * max_vel * sin(angle);
        }

        if (ros::Time::now() - last_warning_flying_slope_time_ > ros::Duration(8.0f)) {
//          uchar flying_slope_warning;
//          if (target_is_red_) {
//            flying_slope_warning = warning_image_.at<uchar>(
//                int(map_pub_list_.array[i].target_position_x * 100),
//                int(map_pub_list_.array[i].target_position_y * 100));
//          }
//          else
//          {
//            flying_slope_warning = warning_image_.at<uchar>(
//                int(2800 - map_pub_list_.array[i].target_position_x * 100),
//                int(1500 - map_pub_list_.array[i].target_position_y * 100));
//          }
          //          std::cout << map_pub_list_.array[i].target_position_x * 100 << ";" << map_pub_list_.array[i].target_position_y * 100 << std::endl;
//          auto color = warning_image_.at<cv::Vec3b>(int(map_pub_list_.array[i].target_position_x * 100), int(map_pub_list_.array[i].target_position_y * 100));
          float max_x_fly_slope = 20.96529f;
          float min_x_fly_slope = 15.38602f;
          float max_y_fly_slope = 15.0f;
          float min_y_fly_slope = 14.01810f;
          if (target_is_red_)
          {
            std::swap(max_x_fly_slope, min_x_fly_slope);
            max_x_fly_slope = 28.0f - max_x_fly_slope;
            min_x_fly_slope = 28.0f - min_x_fly_slope;
            std::swap(max_y_fly_slope, min_y_fly_slope);
            max_y_fly_slope = 15.0f - max_y_fly_slope;
            min_y_fly_slope = 15.0f - min_y_fly_slope;
          }
          float x = map_pub_list_.array[i].target_position_x;
          float y = map_pub_list_.array[i].target_position_y;
          if (x >= min_x_fly_slope && x <= max_x_fly_slope && y >= min_y_fly_slope && y <= max_y_fly_slope) {
            std_msgs::String msg;
            msg.data = robot_name[i] + "_Fly";
            radar_custom_info_pub_.publish(msg);
            last_warning_flying_slope_time_ = ros::Time::now();
          }
        }
        //          radar_warning_state_.warning_flying_slope = true;
//        else
//          radar_warning_state_.warning_flying_slope = false;
      }

      last_pub_time_ = ros::Time::now();
      map_pub_list_.stamp = last_pub_time_;
      radar_test_pub_.publish(map_pub_list_);
      map_last_pub_list_.array.assign(map_pub_list_.array.begin(), map_pub_list_.array.end());

      rm_msgs::ClientMapReceiveData target_msg;
      target_msg.stamp = last_pub_time_;
      target_msg.hero_position_x = int(map_pub_list_.array[0].target_position_x * 100);
      target_msg.hero_position_y = int(map_pub_list_.array[0].target_position_y * 100);
      target_msg.engineer_position_x = int(map_pub_list_.array[1].target_position_x * 100);
      target_msg.engineer_position_y = int(map_pub_list_.array[1].target_position_y * 100);
      target_msg.infantry_3_position_x = int(map_pub_list_.array[2].target_position_x * 100);
      target_msg.infantry_3_position_y = int(map_pub_list_.array[2].target_position_y * 100);
      target_msg.infantry_4_position_x = int(map_pub_list_.array[3].target_position_x * 100);
      target_msg.infantry_4_position_y = int(map_pub_list_.array[3].target_position_y * 100);
      target_msg.infantry_5_position_x = int(map_pub_list_.array[4].target_position_x * 100);
      target_msg.infantry_5_position_y = int(map_pub_list_.array[4].target_position_y * 100);
      target_msg.sentry_position_x = int(map_pub_list_.array[5].target_position_x * 100);
      target_msg.sentry_position_y = int(map_pub_list_.array[5].target_position_y * 100);
      radar_receive_pub_.publish(target_msg);

      current_detected_.assign(ROBOT_NUM, false);
      left_detected_list_.assign(ROBOT_NUM, false);
      right_detected_list_.assign(ROBOT_NUM, false);

//      radar_warning_pub_.publish(radar_warning_state_);
    }
  }
}

void RmTower::initialize(ros::NodeHandle& nh)
{
  ros::NodeHandle nh_missing(nh, "missing_array");
  ros::NodeHandle nh_base_attacked(nh, "base_attacked_missing_array");
  ros::NodeHandle nh_hero_attacked(nh, "Hero_BR3_attacked_missing_array");
  std::vector<std::string> missing_keys{ "hero", "engineer", "robot3", "robot4", "robot5", "sentry" };
  XmlRpc::XmlRpcValue points;
  int id = 0;
  XmlRpc::XmlRpcValue index_array;
  int base_attacked_index = 0;
  nh_base_attacked.getParam("index", index_array);
  std::vector<int> base_attacked_index_array;
  for (int i = 0; i < static_cast<int>(index_array.size()); i++)
    base_attacked_index_array.emplace_back(index_array[i]);

  int hero_attacked_index = 0;
  nh_hero_attacked.getParam("index", index_array);
  std::vector<int> hero_attacked_index_array;
  for (int i = 0; i < static_cast<int>(index_array.size()); i++)
    hero_attacked_index_array.emplace_back(index_array[i]);

  for (auto& key : missing_keys)
  {
    if (nh_missing.getParam(key, points))
      missing_selector_.emplace_back(points, target_is_red_);

    if (base_attacked_index_array[base_attacked_index] == id)
    {
      nh_base_attacked.getParam("data", points);
      missing_selector_[id].setBaseAttackedArray(points, target_is_red_);
      base_attacked_index++;
    }

    if (hero_attacked_index_array[hero_attacked_index] == id)
    {
      nh_hero_attacked.getParam("data", points);
      missing_selector_[id].setHeroAttackedArray(points, target_is_red_);
      hero_attacked_index++;
    }
    id++;
  }
}

void onMouse(int event, int x, int y, int flags, void* param)
{
  rm_tower::RmTower* p_this = ((rm_tower::RmTower*)param);
  switch (event)
  {
    //按下左键
    case cv::EVENT_LBUTTONDOWN:
    {
      p_this->get_rect_ = true;
      p_this->dart_rect_ = cv::Rect(x, y, 0, 0);
      break;
    }
    //鼠标移动
    case cv::EVENT_MOUSEMOVE:
    {
      if (p_this->get_rect_)
      {
        p_this->dart_rect_.width = x - p_this->dart_rect_.x;
        p_this->dart_rect_.height = y - p_this->dart_rect_.y;
        if (p_this->dart_rect_.width < 0)
        {
          p_this->dart_rect_.x -= p_this->dart_rect_.width;
          p_this->dart_rect_.width *= -1;
        }
        if (p_this->dart_rect_.height < 0)
        {
          p_this->dart_rect_.y -= p_this->dart_rect_.height;
          p_this->dart_rect_.height *= -1;
        }
      }
      break;
    }
    //鼠标左键抬起
    case cv::EVENT_LBUTTONUP:
    {
      p_this->get_rect_ = false;
      p_this->dart_get_ = true;
      break;
    }
    default:
      break;
    }
}

}  // namespace rm_tower

PLUGINLIB_EXPORT_CLASS(rm_tower::RmTower, nodelet::Nodelet)