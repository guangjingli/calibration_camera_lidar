//
// Created by lgj on 2019/11/18.
//

#ifndef FUSION_DETECTION_NODELET_TO_IMAGE_H
#define FUSION_DETECTION_NODELET_TO_IMAGE_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>


typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;



class ToImage{
public:
    ToImage();

    void doInit(const std::string& extrinsic_path);

    void point2image(
            const PointType& point_in,
            Eigen::Vector3d& point_image
            );

    void pointcloud2image(
            const PointCloud& point_cloud,
            std::vector<Eigen::Vector3d>& points_image
            );

    void point2image(
            const PointType& point_in,
            cv::Point2f& point_image
            );

    void pointcloud2image(
            const PointCloud& point_cloud,
            std::vector<cv::Point2f>& points_image
            );
private:

    cv::Mat camera_extrinsic_;
    cv::Mat camera_intrinsic_;
    cv::Mat distcoeff_;

    cv::Mat invR_;
    cv::Mat invT_;
    cv::Mat R_;
    cv::Mat t_;
};





#endif //FUSION_DETECTION_NODELET_TO_IMAGE_H
