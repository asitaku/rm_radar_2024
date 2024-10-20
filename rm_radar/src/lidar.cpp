//
// Created by jg on 2022/4/2.
//

#include "lidar.h"

namespace rm_radar
{
PLUGINLIB_EXPORT_CLASS(rm_radar::Lidar, nodelet::Nodelet)

Lidar::Lidar() : target_list_(TARGET_NUM, cv::Point3f(0, 0, 0))
{
  uv_queue_.reserve(PCL_QUEUE_SIZE);  // reserve shared_ptr<PointUV> vector's capacity, in case the memory loss
}

Lidar::~Lidar()
{
  uv_queue_.clear();  // clear vector elements
  if (UI_thread_.joinable())
    UI_thread_.join();  // join other thread
}

void Lidar::onInit()  // start the main program
{
  nh_ = this->getPrivateNodeHandle();

  ros::NodeHandle nh_solvepnp(nh_, "coordinate_solvePnP");

  nh_.getParam("is_game", is_game_);
  MouseCB mouse(nh_);  // using cv::setMouseCallBack to get transform matrix

  getParam(mouse.mouseCB_mat_);  // get camera to world matrix from this object
  ground_tran_mat_ = mouse.ground_tran_mat_;
  highland_tran_mat_ = mouse.highland_tran_mat_;

  //  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(camera_name_ + "output", 1);
  if (nh_.param("left_camera", true))
    target_pub_ = nh_.advertise<rm_msgs::ClientMapDataArray>("/rm_radar_left/detections", 5);
  else
    target_pub_ = nh_.advertise<rm_msgs::ClientMapDataArray>("/rm_radar_right/detections", 5);


//  for (int i = 0; i < TARGET_NUM - 1; i++)
//  {
//    ros::Publisher target_pub =
//        nh_.advertise<rm_msgs::ClientMapReceiveData>("/rm_radar/detection_" + std::to_string(i + 1), 10);
//    target_pub_list_.emplace_back(target_pub);
//  }
//  ros::Publisher target_pub = nh_.advertise<rm_msgs::ClientMapReceiveData>("/rm_radar/detection_7", 10);
//  target_pub_list_.emplace_back(target_pub);

//  for (int i = 0; i < TARGET_NUM; i++)
//  {
//    // publish msg that the UI standby to receive, topic name is: left_camera_proc_target_1 (2, 3, 4, 5)
//    ros::Publisher target_pub_UI =
//        nh_.advertise<std_msgs::Float32MultiArray>(camera_name_ + "_target_" + std::to_string(i + 1), 10);
//    target_pub_list_UI_.emplace_back(target_pub_UI);
//  }

  ros::NodeHandle nh_reconfig(nh_, "rm_radar_reconfig");
  srv_ = new dynamic_reconfigure::Server<LidarConfig>(nh_reconfig);
  dynamic_reconfigure::Server<LidarConfig>::CallbackType cb = boost::bind(&Lidar::reconfigCB, this, _1, _2);
  srv_->setCallback(cb);

  pcl_sub_ =
      nh_.subscribe("/livox/lidar/nonground", 10, &Lidar::cloudCB, this);  // subscribe livox lidar pointCloud message

  // subscribe the msg publish from hk camera (roi msg processed by rm_detector_tensorRT)
  if (nh_.param("left_camera", true))
    detections_sub_ = nh_.subscribe("/left_camera_proc/rm_radar/roi_datas", 10, &Lidar::targetCB, this);
  else
    detections_sub_ = nh_.subscribe("/right_camera_proc/rm_radar/roi_datas", 10, &Lidar::targetCB, this);

//  ec_.setCorePointMinPts(20);

  // test 4. uncomment the following line to test the EuclideanClusterExtraction
  // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  kMeans_ = new KMeans(3, 10);

  cv::Mat image = cv::imread(ros::package::getPath("rm_radar") + "/config/map2024_binary.jpg");
  getWall(image);

  map_mask_ = cv::imread(ros::package::getPath("rm_radar") + "/config/map_mask.jpg");
}

// Get params from .yaml file. Param about camera matrix.
// Including : intrinsic matrix, extrinsic matrix, distortion matrix, projection matrix,
// Also read in cam_to_world matrix from input data: "world_to_cam_MCB", this could let radar know its own position.
void Lidar::getParam(std::vector<float> world_to_cam_MCB)
{
  assert(nh_.getParam("camera_name", camera_name_));  // camera_name location is: radar_double_camera.launch
  bool cam_to_world_identity{};
  assert(nh_.getParam("cam_to_world_identity", cam_to_world_identity));
  std::cout << "camera_name is: " << camera_name_ << std::endl;

  nh_.getParam("target_is_red", target_is_red_);

  int intrinsic_rows = 0;
  int intrinsic_cols = 0;
  nh_.getParam(camera_name_ + "_config/camera_matrix/rows", intrinsic_rows);
  nh_.getParam(camera_name_ + "_config/camera_matrix/cols", intrinsic_cols);
  std::vector<float> intrinsic;
  assert(nh_.getParam(camera_name_ + "_config/camera_matrix/data", intrinsic));
  if (intrinsic.size() != 3 * 3)
    exit(1);
  intrinsic_ = cv::Matx<float, 3, 3>(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5],
                                     intrinsic[6], intrinsic[7], intrinsic[8]);
  std::cout << "intrinsic maxtric is: " << intrinsic_ << std::endl;

  int extrinsic_rows = 0;
  int extrinsic_cols = 0;
  nh_.getParam(camera_name_ + "_config/extrinsic/rows", extrinsic_rows);
  nh_.getParam(camera_name_ + "_config/extrinsic/cols", extrinsic_cols);
  std::vector<float> extrinsic(extrinsic_rows * extrinsic_cols);
  nh_.getParam(camera_name_ + "_config/extrinsic/data", extrinsic);

  if (extrinsic.size() != 3 * 4)
    exit(1);
  extrinsic_ =
      cv::Matx<float, 3, 4>(extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3], extrinsic[4], extrinsic[5],
                            extrinsic[6], extrinsic[7], extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]);
  // the extrinsic may update with dynamic-reconfig, use this matrix to save the initial extrinsic
  extrinsic_temp_ = extrinsic_;
  std::cout << "extrinsic maxtric is: " << extrinsic_ << std::endl;

  extrinsic_r_ = cv::Matx<float, 3, 3>(extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[4], extrinsic[5],
                                       extrinsic[6], extrinsic[8], extrinsic[9], extrinsic[10]);

  extrinsic_t_ = cv::Matx<float, 3, 1>(extrinsic[3], extrinsic[7], extrinsic[11]);

  in_dot_ex_ = intrinsic_ * extrinsic_;
  std::cout << "in_dot_ex is: " << in_dot_ex_ << std::endl;
  cv::Matx31f test_answer = in_dot_ex_ * cv::Matx<float, 4, 1>(132, 12, 23, 1);
  std::cout << "in_dot_ex * test_matrix from world is: " << test_answer << std::endl;

  int distortion_rows = 0;
  int distortion_cols = 0;
  nh_.getParam(camera_name_ + "_config/distortion_coefficients/rows", distortion_rows);
  nh_.getParam(camera_name_ + "_config/distortion_coefficients/cols", distortion_cols);
  std::vector<float> distortion;
  nh_.getParam(camera_name_ + "_config/distortion_coefficients/data", distortion);
  if (distortion.size() != 1 * 5)
    exit(1);
  distortion_ = cv::Matx<float, 1, 5>(distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]);

  int projection_rows = 0;
  int projection_cols = 0;
  nh_.getParam(camera_name_ + "_config/projection_matrix/rows", projection_rows);
  nh_.getParam(camera_name_ + "_config/projection_matrix/cols", projection_cols);
  std::vector<float> projection(extrinsic_rows * projection_cols);
  nh_.getParam(camera_name_ + "_config/projection_matrix/data", projection);
  if (projection.size() != 3 * 4)
    exit(1);
  projection_ =
      cv::Matx<float, 3, 4>(projection[0], projection[1], projection[2], projection[3], projection[4], projection[5],
                            projection[6], projection[7], projection[8], projection[9], projection[10], projection[11]);

  //  Code down here is for getting world_to_cam matrix from .yaml,
  //  but now we don't have to, so those code down here is abandoned.
  //  we are now using Mouse Callback function and solvePnP to get the world_to_cam matrix
  //  int world_to_cam_rows = 0;
  //  int world_to_cam_cols = 0;
  //  nh_.getParam(camera_name_ + "_config/world_to_cam/rows", world_to_cam_rows);
  //  nh_.getParam(camera_name_ + "_config/world_to_cam/cols", world_to_cam_cols);
  //  std::vector<double> world_to_cam(world_to_cam_rows * world_to_cam_cols);
  //  nh_.getParam(camera_name_ + "_config/world_to_cam/data", world_to_cam);
  //  if (world_to_cam.size() != 3 * 4)
  //    exit(1);
  //    cam_to_world_ = cv::Matx<float, 4, 4>(
  //                world_to_cam[0],world_to_cam[1],world_to_cam[2],world_to_cam[3],
  //                world_to_cam[4],world_to_cam[5],world_to_cam[6],world_to_cam[7],
  //                world_to_cam[8],world_to_cam[9],world_to_cam[10],world_to_cam[11],
  //                0, 0, 0, 1
  //    ).inv(); // invert the matrix

  cam_to_world_ = cv::Matx<float, 4, 4>(world_to_cam_MCB[0], world_to_cam_MCB[1], world_to_cam_MCB[2],
                                        world_to_cam_MCB[3], world_to_cam_MCB[4], world_to_cam_MCB[5],
                                        world_to_cam_MCB[6], world_to_cam_MCB[7], world_to_cam_MCB[8],
                                        world_to_cam_MCB[9], world_to_cam_MCB[10], world_to_cam_MCB[11], 0, 0, 0, 1)
                      .inv();  // invert the matrix(逆矩阵)
  if (cam_to_world_identity)
    cam_to_world_ = cv::Matx<float, 4, 4>(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);
  std::cout << "cam_to_world_matrix_inverted from solvePnP transport to here is: " << cam_to_world_ << std::endl;
//  if (camera_name_.compare("right_camera_proc") == 0)
//  {
//    cv::Matx44f extrinsic_right, extrinsic_left;
//    extrinsic_right =
//        cv::Matx<float, 4, 4>(extrinsic_.val[0], extrinsic_.val[1], extrinsic_.val[2], extrinsic_.val[3],
//                              extrinsic_.val[4], extrinsic_.val[5], extrinsic_.val[6], extrinsic_.val[7],
//                              extrinsic_.val[8], extrinsic_.val[9], extrinsic_.val[10], extrinsic_.val[11], 0, 0, 0, 1);
//    std::vector<float> extrinsic_left_temp(4 * 4);
//    nh_.getParam(camera_name_ + "_config/extrinsic_left/data", extrinsic_left_temp);
//    extrinsic_left =
//        cv::Matx<float, 4, 4>(extrinsic_left_temp[0], extrinsic_left_temp[1], extrinsic_left_temp[2],
//                              extrinsic_left_temp[3], extrinsic_left_temp[4], extrinsic_left_temp[5],
//                              extrinsic_left_temp[6], extrinsic_left_temp[7], extrinsic_left_temp[8],
//                              extrinsic_left_temp[9], extrinsic_left_temp[10], extrinsic_left_temp[11], 0, 0, 0, 1);
//    cam_to_world_ = extrinsic_right.inv() * extrinsic_left * cam_to_world_;
//  }

  std::cout << "cam_to_world_matrix_inverted real is: " << cam_to_world_ << std::endl;
}

// Get msg from pointCloud topic, when get msg from topic, auto activate this callBack function
void Lidar::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud_xyz);

  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  //  filter_.setInputCloud(cloud_xyz);
  //  filter_.setRadiusSearch(radius_search_);
  //  filter_.setMinNeighborsInRadius(min_neighbors_in_radius_);
  //  filter_.filter(*cloud_filtered);

//  boost::shared_ptr<std::vector<PointUV>> uv_with_depth(new std::vector<PointUV>);
//  getUVDepth(cloud_xyz, uv_with_depth);  // creating depth image
  boost::shared_ptr<std::vector<PointXYZ_UV>> xyz_uz(new std::vector<PointXYZ_UV>);
  getUVDepth(cloud_xyz, xyz_uz);
  {
    boost::unique_lock<boost::shared_mutex> write_mutex(pcl_queue_lock_);
//    if (uv_queue_.size() >= PCL_QUEUE_SIZE)
//      uv_queue_.erase(uv_queue_.cbegin());
//    uv_queue_.emplace_back(uv_with_depth);  // point getUVdepth from pointCloud
//    xyz_queue_.emplace_back(cloud_xyz);
    if (xyz_uv_queue_.size() >= PCL_QUEUE_SIZE)
      xyz_uv_queue_.erase(xyz_uv_queue_.cbegin());
    xyz_uv_queue_.emplace_back(xyz_uz);
  }
}

// Get data from dynamic reconfigure
void Lidar::reconfigCB(LidarConfig& config, uint32_t level)
{
  (void)level;  // (void)variable means we don's need this variable, set it into (void) in case get bug
  if (!initialized_flag_)
  {
    config.show_cam_to_lidar_test = show_cam_to_lidar_test_;
    config.show_cam_to_world_test = show_cam_to_world_test_;
    initialized_flag_ = true;
  }

  radius_search_ = config.radius_search;
  min_neighbors_in_radius_ = config.min_neighbors_in_radius;

  rot_x_ = cv::Matx<float, 4, 4>(1, 0, 0, 0, 0, cos(config.rot_x * PI / 180), -sin(config.rot_x * PI / 180), 0, 0,
                                 sin(config.rot_x * PI / 180), cos(config.rot_x * PI / 180), 0, 0, 0, 0, 1);
  rot_y_ = cv::Matx<float, 4, 4>(cos(config.rot_y * PI / 180), 0, sin(config.rot_y * PI / 180), 0, 0, 1, 0, 0,
                                 -sin(config.rot_y * PI / 180), 0, cos(config.rot_y * PI / 180), 0, 0, 0, 0, 1);
  rot_z_ =
      cv::Matx<float, 4, 4>(cos(config.rot_z * PI / 180), -sin(config.rot_z * PI / 180), 0, 0,
                            sin(config.rot_z * PI / 180), cos(config.rot_z * PI / 180), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  trans_ = cv::Matx<float, 4, 4>(1, 0, 0, config.trans_x, 0, 1, 0, config.trans_y, 0, 0, 1, config.trans_z, 0, 0, 0, 1);
  extrinsic_1_ = trans_ * rot_x_ * rot_y_ * rot_z_;
  extrinsic_ = extrinsic_temp_ * extrinsic_1_;
  in_dot_ex_ = intrinsic_ * extrinsic_;
  extrinsic_r_ = cv::Matx<float, 3, 3>(extrinsic_.val[0], extrinsic_.val[1], extrinsic_.val[2], extrinsic_.val[4],
                                       extrinsic_.val[5], extrinsic_.val[6], extrinsic_.val[8], extrinsic_.val[9],
                                       extrinsic_.val[10]);
  extrinsic_t_ = cv::Matx<float, 3, 1>(extrinsic_.val[3], extrinsic_.val[7], extrinsic_.val[11]);
  is_extrinsic_ = config.extrinsic;
  show_cam_to_lidar_test_ = config.show_cam_to_lidar_test;
  show_cam_to_world_test_ = config.show_cam_to_world_test;
}

// get enemy (x, y, z) information, and publish to UI node, UI is like a mini-map in game, show detected enemy position
void Lidar::targetCB(const rm_msgs::RadarTargetDetectionArray::ConstPtr& roi_msg)
{
  rm_msgs::ClientMapDataArray data_msg;
  data_msg.stamp = roi_msg->header.stamp;
  for (const auto& detection : roi_msg->detections)
  {
    cv::Matx22f roi_points = cv::Matx<float, 2, 2>(detection.position.data[0], detection.position.data[1],
                                                   detection.position.data[2], detection.position.data[3]);
    cv::Point3f target(0, 0, 0);
    //  judge it's the true enemy or not, if true, publish xyz message
    //    std_msgs::Float32MultiArray target_msg_UI{};
    if (getTarget(&roi_points, target))
    {
      rm_msgs::ClientMapData target_msg{};
      target_msg.stamp = roi_msg->header.stamp;
      if (target_is_red_)
      {
        target_msg.target_robot_ID = detection.id - 6;
        target_msg.target_position_x = abs(28.0f - target.x);
        target_msg.target_position_y = abs(15.0f - target.y);
      }
      else
      {
        target_msg.target_robot_ID = detection.id;
        target_msg.target_position_x = target.x;
        target_msg.target_position_y = target.y;
      }
//      target_msg.stamp = ros::Time::now();
//      target_pub_list_[std::min(TARGET_NUM - 1, target_msg.target_robot_ID % 10 - 1)].publish(target_msg);
      data_msg.array.emplace_back(target_msg);

      //      ROS_INFO(("target_" + std::to_string(detection.id) + "(%f,%f,%f)").c_str(), target.x, target.y, target.z);

      if (show_cam_to_world_test_)
        target_broadcaster_.sendTransform(
            tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(target.x, target.y, target.z)),
                                 ros::Time::now(), "livox_frame", "target_" + std::to_string(detection.id)));
    }
  }
  target_pub_.publish(data_msg);
}

// judge the object is target or not
bool Lidar::getTarget(cv::Matx22f* roi_points, cv::Point3f& target)
{
  if (xyz_uv_queue_.size() >= PCL_QUEUE_SIZE)
  {
    if (roi_points->dot(*roi_points) != 0)
    {
      std::vector<cv::Point2f> roi_points_vec;
      cv::Point2f roi_tl(roi_points->val[0], roi_points->val[1]);
      cv::Point2f roi_br(roi_points->val[2], roi_points->val[3]);
      roi_points_vec.emplace_back(roi_tl);
      roi_points_vec.emplace_back(roi_br);

      float center_x = (roi_tl.x + roi_br.x) / 2;
      float center_y = (roi_tl.y + roi_br.y) / 2 + (roi_br.y - roi_tl.y) * 0.0f;
      cv::undistortPoints(roi_points_vec, roi_points_vec, intrinsic_, distortion_, cv::noArray(), projection_);

//      double depth = 0;
//      int sum = 0;
      cv::Point2f tl(std::min(roi_points_vec[0].x, roi_points_vec[1].x),
                     std::min(roi_points_vec[0].y, roi_points_vec[1].y));
      cv::Point2f br(std::max(roi_points_vec[0].x, roi_points_vec[1].x),
                     std::max(roi_points_vec[0].y, roi_points_vec[1].y));
      // maybe we need to adjust the top_left and bottom_right position, in case the tl is br.
      // make the target rect smaller
      float xd = (br.x - tl.x) * 0.2f;
      float yd = (br.y - tl.y) * 0.2f;
//      tl.x = tl.x + xd;
//      tl.y = tl.y + yd;
//      br.x = br.x - xd;
//      br.y = br.y - yd - yd * 0.0f;

      std::vector<cv::Point2f> target_2d_array;
      target_2d_array.emplace_back(center_x, center_y);
      std::vector<cv::Point2f> target_array;
      cv::perspectiveTransform(target_2d_array, target_array, ground_tran_mat_);
      cv::Point3f target_ground = cv::Point3f(target_array[0].x / 100, target_array[0].y / 100, 0.0f);
      cv::Matx41f target_in_cam = cam_to_world_.inv() * cv::Matx41f(target_ground.x, target_ground.y, target_ground.z, 1);
      PointUV target_uv = coordinateCamToUV(cv::Point3f(target_in_cam.val[0], target_in_cam.val[1], target_in_cam.val[2]));

      tl.x = target_uv.u - xd;
      tl.y = target_uv.v - yd;
      br.x = target_uv.u + xd;
      br.y = target_uv.v + yd;
//      PointUV target_depth;
//      std::vector<PointUV> cloud_selected;
//      pcl::PointCloud<pcl::PointXYZ> xyz_selected;
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_selected;
      std::vector<PointXYZ_UV> cloud_selected;
//      std::vector<float> depth_v;
            //      PointUV target_depth;
      int num = 0;

      cv::Point3f binary_point;
      cv::Matx41f binary_mat;

      {
        boost::shared_lock<boost::shared_mutex> read_mutex(pcl_queue_lock_);
        for (auto& i : xyz_uv_queue_)  // todo kdtree
        {
          for (auto& j : *i)
          {
            if (j.uv.u > tl.x && j.uv.u < br.x)
              if (j.uv.v > tl.y && j.uv.v < br.y)
                if (j.uv.depth < 35.0f && j.uv.depth > 4.0f)
                {
                  binary_point = coordinateUVToCam(j.uv);
                  binary_mat = cam_to_world_ * cv::Matx41f(binary_point.x, binary_point.y, binary_point.z, 1);
                  if (binary_mat.val[0] <= 0.0f || binary_mat.val[0] >= 28.0f || binary_mat.val[1] <= 0.0f || binary_mat.val[1] >= 15.0f || binary_mat.val[2] >= 2.5f || binary_mat.val[2] <= 0.05f ||
                      map_binary_.at<uchar>(int((binary_mat.val[0] + 1.5f) / 0.029f), int((binary_mat.val[1] + 1.5f) / 0.029f)) != 255 || abs(binary_mat.val[2] - 0.4f) <= 0.02f || abs(binary_mat.val[2] - 0.6f) <= 0.02f)
                    continue;
                  num++;
                  pcl::PointXYZ xyz;
                  xyz.x = binary_mat.val[0];
                  xyz.y = binary_mat.val[1];
                  xyz.z = binary_mat.val[2];
                  j.xyz = xyz;
                  cloud_selected.push_back(j);
                }
          }
        }
      }

//      int map_mask_x = 2800 - int(target.x * 100);
//      int map_mask_y = 1500 - int(target.y * 100);
//      auto map_mask = map_mask_.at<cv::Vec3b>(map_mask_x, map_mask_y).val;
//      if ((map_mask[0] != 0 || map_mask[1] != 0 || map_mask[2] != 0))
//      {
      cv::perspectiveTransform(target_2d_array, target_array, highland_tran_mat_);
      cv::Point3f target_highland = cv::Point3f(target_array[0].x / 100, target_array[0].y / 100, 0.6f);
//      }

      if (!num)
      {
        target = cv::Point3f((target_ground.x + target_highland.x) / 2, (target_ground.y + target_highland.y) / 2, (target_ground.z + target_highland.z) / 2);
        return true;
      }

      std::sort(cloud_selected.begin(), cloud_selected.end(), [=](PointXYZ_UV& a, PointXYZ_UV& b){
//        return (a.uv.u - target_uv.u) * (a.uv.u - target_uv.u) + (a.uv.v - target_uv.v) * (a.uv.v - target_uv.v) < (b.uv.u - target_uv.u) * (b.uv.u - target_uv.u) + (b.uv.v - target_uv.v) * (b.uv.v - target_uv.v);
        return a.uv.depth < b.uv.depth;
      });

//      if (cloud_selected.at(0).xyz.x > target.x)
//        target = cv::Point3f(target_2d.x, target_2d.y, 0.0f);

      kMeans_->centre_points_.clear();
      kMeans_->centre_points_.emplace_back(cloud_selected.at(0).xyz);
      kMeans_->centre_points_.emplace_back(target_ground.x, target_ground.y, target_ground.z);
      kMeans_->centre_points_.emplace_back(target_highland.x, target_highland.y, target_highland.z);
      pcl::PointCloud<pcl::PointXYZ> target_xyz = kMeans_->kMeans(cloud_selected);

//      int min_x_index = 0;
//      for (int i = 1; i < 3; i++)
//        if (target_xyz.at(i).x < target_xyz.at(min_x_index).x)
//          min_x_index = i;
      int target_index = 1;
      if (target_xyz.at(0).z >= 0.5)
        target_index = 0;
      std::sort(target_xyz.begin(), target_xyz.end(), [](pcl::PointXYZ& a, pcl::PointXYZ& b){
        return a.x < b.x;
      });
      target = cv::Point3f(target_xyz.at(target_index).x, target_xyz.at(target_index).y, target_xyz.at(target_index).z);
//      if (target.x > target_xyz.at(0).x)
//        target = cv::Point3f(target_xyz.at(0).x, target_xyz.at(0).y, target_xyz.at(0).z);
//      std::reverse(target_xyz.begin(), target_xyz.end());

//      for (auto i : target_xyz)
//      {
//        if (i.x <= target.x)
//        {
//          target = cv::Point3f(i.x, i.y, i.z);
//          break;
//        }
//      }

      return true;
    }
  }
  return false;
}

void Lidar::coordinateCamToLidar(const cv::Point3f target_in_cam_p)
{
  cv::Matx41f target_in_cam = { target_in_cam_p.x, target_in_cam_p.y, target_in_cam_p.z, 1 };
  cv::Matx41f target_in_lidar;
  cv::Matx44f extrinsic;
  extrinsic =
      cv::Matx<float, 4, 4>(extrinsic_.val[0], extrinsic_.val[1], extrinsic_.val[2], extrinsic_.val[3],
                            extrinsic_.val[4], extrinsic_.val[5], extrinsic_.val[6], extrinsic_.val[7],
                            extrinsic_.val[8], extrinsic_.val[9], extrinsic_.val[10], extrinsic_.val[11], 0, 0, 0, 1);
  target_in_lidar = extrinsic.inv() * target_in_cam;
  std_msgs::Float32MultiArray target_msg;
  target_msg.data = { target_in_lidar(0), target_in_lidar(1), target_in_lidar(2) };
  target_broadcaster_.sendTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(target_in_lidar(0), target_in_lidar(1), target_in_lidar(2))),
      ros::Time::now(), "livox_frame", "target_" + std::to_string(1)));
}

//  convert coordination from UV to camera, may be no need to change
cv::Point3f Lidar::coordinateUVToCam(const PointUV& uv_depth)
{
  cv::Point3f target_in_cam;
  target_in_cam.x = (uv_depth.u - projection_(0, 2) - projection_(0, 3)) / projection_(0, 0) * uv_depth.depth;
  target_in_cam.y = (uv_depth.v - projection_(1, 2) - projection_(1, 3)) / projection_(1, 1) * uv_depth.depth;
  target_in_cam.z = 1.0 * uv_depth.depth;
  return target_in_cam;
}

// this camToWorld function may be abandoned
cv::Point3f Lidar::coordinateCamToWorld(const cv::Point3f& target_in_cam)
{
  cv::Point3f target_in_world;
  cv::Matx41f xyz_w, xyz_c;
  xyz_c = cv::Matx<float, 4, 1>(target_in_cam.x, target_in_cam.y, target_in_cam.z, 1);
  cv::Matx44f extrinsic;
  if (is_extrinsic_)
  {
    extrinsic = extrinsic_1_;
  }
  else
  {
    extrinsic =
        cv::Matx<float, 4, 4>(extrinsic_.val[0], extrinsic_.val[1], extrinsic_.val[2], extrinsic_.val[3],
                              extrinsic_.val[4], extrinsic_.val[5], extrinsic_.val[6], extrinsic_.val[7],
                              extrinsic_.val[8], extrinsic_.val[9], extrinsic_.val[10], extrinsic_.val[11], 0, 0, 0, 1);
  }
  xyz_w = extrinsic.inv() * xyz_c;
  target_in_world.x = xyz_w(0, 0);
  target_in_world.y = xyz_w(1, 0);
  target_in_world.z = xyz_w(2, 0);
  return target_in_world;
}

//  get image depth information
void Lidar::getUVDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const boost::shared_ptr<std::vector<PointUV>>& uv_with_depth)
{
  for (auto& i : *cloud)
  {
    cv::Matx31f uv_depth = in_dot_ex_ * cv::Matx<float, 4, 1>(i.x, i.y, i.z, 1);
    PointUV point;
    point.u = uv_depth.val[0] / uv_depth.val[2];
    point.v = uv_depth.val[1] / uv_depth.val[2];
    point.depth = uv_depth.val[2];

    uv_with_depth->emplace_back(point);
    // using radar with camera's in_dot_ex matrix to get depth image.
    // this function changed from OpenCV function, said by zhenjie.Zhou
  }
}

void Lidar::getWall(cv::Mat& image)
{
  cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
  cv::threshold(image, image, 200, 255, cv::THRESH_BINARY);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
  cv::erode(image, image, element);

  map_binary_ = image.clone();
}
void Lidar::getUVDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const boost::shared_ptr<std::vector<PointXYZ_UV>>& xyz_uv)
{
  for (auto i : *cloud)
  {
    cv::Matx31f uv_depth = in_dot_ex_ * cv::Matx<float, 4, 1>(i.x, i.y, i.z, 1);
    PointUV point;
    point.u = uv_depth.val[0] / uv_depth.val[2];
    point.v = uv_depth.val[1] / uv_depth.val[2];
    point.depth = uv_depth.val[2];

    PointXYZ_UV temp;
    temp.xyz = i;
    temp.uv = point;


    xyz_uv->emplace_back(temp);
    // using radar with camera's in_dot_ex matrix to get depth image.
    // this function changed from OpenCV function, said by zhenjie.Zhou
  }
}

void Lidar::getUVDepth(pcl::PointCloud<pcl::PointXYZ>& xyz, std::vector<PointUV>& uv)
{
  for (auto i : xyz)
  {
    cv::Matx31f uv_depth = in_dot_ex_ * cv::Matx<float, 4, 1>(i.x, i.y, i.z, 1);
    PointUV point;
    point.u = uv_depth.val[0] / uv_depth.val[2];
    point.v = uv_depth.val[1] / uv_depth.val[2];
    point.depth = uv_depth.val[2];

    uv.emplace_back(point);
    // using radar with camera's in_dot_ex matrix to get depth image.
    // this function changed from OpenCV function, said by zhenjie.Zhou
  }
}

PointUV Lidar::coordinateCamToUV(const cv::Point3f& cam_point)
{
  PointUV target_in_uv;
  target_in_uv.u = cam_point.x * projection_(0, 0) / cam_point.z + projection_(0, 2) + projection_(0, 3);
  target_in_uv.v = cam_point.y * projection_(1, 1) / cam_point.z + projection_(1, 2) + projection_(1, 3);
  target_in_uv.depth = 1.0f * cam_point.z;
  return target_in_uv;
}

bool MouseCB::exit_flag{};

}  // namespace rm_radar

// Down here is mouse callback code
void rm_radar::onMouse(int event, int x, int y, int __attribute__((unused)) flags, void* param)
{
  rm_radar::MouseCB* p_this = ((rm_radar::MouseCB*)param);
  static int count = 1;
  if (event == cv::EVENT_LBUTTONDOWN)
  {
//    if (!p_this->parent_nh_.param("left_camera", true))
//      points_size = 7;
    // get the points from mouse clicked
    std::cout << "Point" << std::to_string(count++) << ": " << cv::Point2d(x, y) << std::endl;
    p_this->points_2d_.emplace_back(cv::Point2d(x, y));
    if (p_this->points_2d_.size() < p_this->points_size_)
      return;

//    if (!p_this->parent_nh_.param("left_camera", true))
//    {
//      p_this->parent_nh_.setParam("dart_x", p_this->points_2d_[6].x);
//      p_this->parent_nh_.setParam("dart_y", p_this->points_2d_[6].y);
//      p_this->points_2d_.pop_back();
//    }

    cv::destroyAllWindows();
    std::cout << "===========================\n";

    std::vector<cv::Point3d> map_points;
    cv::Mat_<double> r_vec(3, 1);
    cv::Mat_<double> t_vec(3, 1);

//    std::vector<cv::Point2f> map_points;
//    for (int i = 6; i < points_size; i++)
//      map_points.push_back(p_this->points_2d_[i]);
//    for (int i = 0; i < 4; i++)
//      p_this->points_2d_.pop_back();
//    ROS_INFO("0");
    if (!p_this->is_game_)
    {
      ros::NodeHandle nh_test(p_this->parent_nh_, "test_points");
      std::vector<std::string> test_keys{ "p1", "p2", "p3", "p4", "p5", "p6", "p7", "p8", "p9", "p10" };
      XmlRpc::XmlRpcValue points;
      for (auto& key : test_keys)
      {
        if (nh_test.getParam(key, points))
        {
          double x_test = points[0];
          double y_test = points[1];
          double z_test = points[2];
          map_points.emplace_back(cv::Point3d(x_test, y_test, z_test));
        }
      }
    }
    else
    {
      ros::NodeHandle nh_game(p_this->parent_nh_, "game_points");
      std::vector<std::string> game_keys{ "p1", "p2", "p3", "p4", "p5", "p6", "p7", "p8", "p9", "p10" };
      XmlRpc::XmlRpcValue points;
      for (auto& key : game_keys)
      {
        if (nh_game.getParam(key, points))
        {
          double x_game = points[0];
          double y_game = points[1];
          double z_game = points[2];
          map_points.emplace_back(cv::Point3d(x_game, y_game, z_game));
        }
      }
      ROS_INFO("1");
      std::vector<cv::Point2f> ground_image_points;
      std::vector<cv::Point2f> highland_image_points;
      std::vector<cv::Point2f> ground_points;
      std::vector<cv::Point2f> highland_points;
      std::vector<int> ground_index;
      std::vector<int> highland_index;
      if (p_this->parent_nh_.param("left_camera", true))
      {
        ground_index = std::vector<int>{ 2, 4, 8, 9 };
        highland_index = std::vector<int>{ 0, 3, 5, 6 };
      }
      else
      {
        ground_index = std::vector<int>{ 1, 3, 5, 7 };
        highland_index = std::vector<int>{ 0, 2, 8, 9 };
      }
      for (int i = 0; i < 4; i++)
      {
        ground_image_points.emplace_back(p_this->points_2d_[ground_index[i]]);
        highland_image_points.emplace_back(p_this->points_2d_[highland_index[i]]);
        ground_points.emplace_back(map_points[ground_index[i]].x * 100, map_points[ground_index[i]].y * 100);
        highland_points.emplace_back(map_points[highland_index[i]].x * 100, map_points[highland_index[i]].y * 100);
      }
      ROS_INFO("get_map");
      p_this->ground_tran_mat_ = cv::getPerspectiveTransform(ground_image_points, ground_points);
      p_this->highland_tran_mat_ = cv::getPerspectiveTransform(highland_image_points, highland_points);
      ROS_INFO("tran");
    }

    cv::solvePnP(map_points, p_this->points_2d_, p_this->cam_intrinsic_mat_k_, p_this->dist_coefficients_,
                 r_vec, t_vec, false, cv::SOLVEPNP_EPNP);

    cv::Rodrigues(r_vec, r_vec);

    cv::Mat trans_mat;
    cv::hconcat(r_vec, t_vec, trans_mat);  // connect two matrix with the same rows and cols

    std::cout << "trans_mat  through solvepnp is: " << trans_mat << std::endl;

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        p_this->mouseCB_mat_.emplace_back(trans_mat.at<double>(i, j));
      }
    }
    MouseCB::exit_flag = true;
  }
}

// Mouse callBack main initialize and execute code.
rm_radar::MouseCB::MouseCB(ros::NodeHandle& nh)
{
  initialize(nh);
  execute();
}

//  read in image information which used to get cam_to_world matrix with Mouse callback function
void rm_radar::MouseCB::initialize(ros::NodeHandle& nh)
{
  parent_nh_ = nh;
  std::string camera_info_name;
  nh.getParam("cameraInfo_yaml_file", camera_info_name);
  nh.getParam("is_game", is_game_);
  if (is_game_)
    points_size_ = 10;
  else
    points_size_ = 6;

  camera_info_manager::CameraInfoManager info_manager{ nh };
  info_manager.loadCameraInfo(camera_info_name);
  camera_info_ = info_manager.getCameraInfo();
  cam_intrinsic_mat_k_.create(3, 3);
  memcpy(cam_intrinsic_mat_k_.data, camera_info_.K.data(), 9 * sizeof(double));

  ROS_ASSERT(cv::determinant(cam_intrinsic_mat_k_) != 0);
  dist_coefficients_ = camera_info_.D;
  ROS_INFO("Successfully read camera info.");

  std::string img = this->template getParam<std::string>(parent_nh_, "img_path", "");
  img_ = cv::imread(img);
  points_3d_.clear();
  points_2d_.clear();
  ROS_INFO("Successfully read input img.");

  // sub image from camera in order to calculate the mouseCB matrix
  if (nh.param("left_camera", true))
    sub_img_ = nh.subscribe("/hk_camera_left/image_raw/compressed", 1, &MouseCB::imgSub, this);
  else
    sub_img_ = nh.subscribe("/hk_camera_right/image_raw/compressed", 1, &MouseCB::imgSub, this);
}

// start to use Mouse callback function
void rm_radar::MouseCB::execute()
{
  parent_nh_.getParam("camera_name", camera_name_);
  cv::namedWindow(camera_name_ + "input_img", cv::WINDOW_KEEPRATIO);
  cv::setMouseCallback(camera_name_ + "input_img", rm_radar::onMouse, (void*)this);
  exit_flag = false;
  // show the image from camera
  while (!exit_flag)
  {
    cv::imshow(camera_name_ + "input_img", img_);
    cv::waitKey(10);
  }
  sub_img_.shutdown();  // shutdown the subscriber
  //  cv::waitKey(10000);  // limited time to use mouse click the image,must be finished within 15s, you can change the time if needed
}

void rm_radar::MouseCB::imgSub(const sensor_msgs::CompressedImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
  cv_ptr->image.copyTo(img_);
}
