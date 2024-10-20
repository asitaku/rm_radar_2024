cmake_minimum_required(VERSION 3.0.2)
project(rm_radar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosmsg
        rospy
        nodelet
        dynamic_reconfigure
        image_transport
        message_generation
        pcl_conversions
        pcl_ros
        sensor_msgs
        cv_bridge
        camera_info_manager
        pluginlib
        rm_msgs
        image_geometry
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/lidar.cfg
)

catkin_package(
        LIBRARIES rm_radar
        CATKIN_DEPENDS nodelet roscpp sensor_msgs dynamic_reconfigure pluginlib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/lidar.cpp src/UI.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})

add_library(UI src/UI.cpp)
target_link_libraries(UI
        ${catkin_LIBRARIES}
)


#
#add_executable(${PROJECT_NAME}_node src/lidar_node.cpp )
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(${PROJECT_NAME}_node
#        ${catkin_LIBRARIES})
