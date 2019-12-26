cmake_minimum_required(VERSION 2.8.3)
project(project_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O2)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
        message_generation
        pcl_conversions
        pcl_ros
        std_msgs
        sensor_msgs
        cv_bridge
        dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/adjust_extrinsic.cfg
)


catkin_package(
        CATKIN_DEPENDS rospy
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(project_pointcloud_node
        src/project_pointcloud.cpp
        src/to_image.cpp
        src/utils.cpp
        )

add_dependencies(project_pointcloud_node project_pointcloud_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(project_pointcloud_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBS}
        )


add_executable(test_node
        src/test.cpp)

target_link_libraries(test_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBS}
        )

