//
// Created by lgj on 2019/12/17.
//

#ifndef CALIBRATION_LIDAR_CAMERA_UTILS_H
#define CALIBRATION_LIDAR_CAMERA_UTILS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

const double degree_per_rad = 180.0 / M_PI;
const double rad_per_degree = M_PI / 180.0;
const double M_2PI = M_PI * 2;
const double M_3PI = M_PI * 3;



int angle_normalized(
        double& angle,
        const double& angle_min = 0.0
);

int fromQuatToEular(
        const geometry_msgs::Quaternion& rot,
        double *yaw,
        double *pitch,
        double *roll
);

cv::Mat fromHprToMat(
        const double& heading,
        const double& pitch,
        const double& roll,
        const double& delta_x,
        const double& delta_y,
        const double& delta_z
);

template <class T>
T BytesTo(const std::vector<uint8_t>& data, uint32_t start_idx){
    const size_t kNumberOfBytes = sizeof(T);
    uint8_t byte_array[kNumberOfBytes];
    for(size_t i = 0; i < kNumberOfBytes; ++i){
        byte_array[i] = data[start_idx + i];
    }
    T result;
    std::copy(reinterpret_cast<const uint8_t *>(&byte_array[0]),
              reinterpret_cast<const uint8_t *>(&byte_array[kNumberOfBytes]),
              reinterpret_cast<uint8_t *>(&result));
    return result;
}
#endif //CALIBRATION_LIDAR_CAMERA_UTILS_H
