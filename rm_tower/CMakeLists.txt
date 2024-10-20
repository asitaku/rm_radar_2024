cmake_minimum_required(VERSION 3.0.2)
project(rm_tower)


find_package(catkin REQUIRED COMPONENTS
        rm_msgs
        roscpp
        rospy
        sensor_msgs
        nodelet
        pluginlib
        rm_common
        cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES rm_tower
        CATKIN_DEPENDS rm_msgs roscpp rospy sensor_msgs pluginlib nodelet rm_common cv_bridge
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/rm_tower.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

