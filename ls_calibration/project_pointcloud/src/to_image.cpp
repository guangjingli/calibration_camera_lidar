//
// Created by lgj on 2019/11/18.
//

#include "to_image.h"
#include "singleton.hpp"

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m01 = rotationMatrix.at<double>(0, 1);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m21 = rotationMatrix.at<double>(2, 1);
    double m22 = rotationMatrix.at<double>(2, 2);

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.9999) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    }
    else if (m10 < -0.9999) { // singularity at south pole
        x = 0;
        y = -CV_PI / 2;
        z = atan2(m02, m22);
    }
    else
    {
        x = atan2(m21, m22);
        y = atan2(-m20, sqrt(m21*m21 + m22*m22));
        z = atan2(m10, m00);
//        x = atan2(-m12, m11);
//        y = asin(m10);
//        z = atan2(-m20, m00);
    }

    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;

    return euler;
}

void euler2rot(const cv::Mat euler, cv::Mat& lidar_extrinsic, ExtrinsicConfig config){
    Eigen::Matrix3d rotation;
    rotation =
            Eigen::AngleAxisd(euler.at<double>(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler.at<double>(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler.at<double>(0), Eigen::Vector3d::UnitX());

    lidar_extrinsic.at<double>(0, 0) = rotation(0, 0);
    lidar_extrinsic.at<double>(0, 1) = rotation(0, 1);
    lidar_extrinsic.at<double>(0, 2) = rotation(0, 2);
    lidar_extrinsic.at<double>(1, 0) = rotation(1, 0);
    lidar_extrinsic.at<double>(1, 1) = rotation(1, 1);
    lidar_extrinsic.at<double>(1, 2) = rotation(1, 2);
    lidar_extrinsic.at<double>(2, 0) = rotation(2, 0);
    lidar_extrinsic.at<double>(2, 1) = rotation(2, 1);
    lidar_extrinsic.at<double>(2, 2) = rotation(2, 2);

    lidar_extrinsic.at<double>(0, 3) += config.x;
    lidar_extrinsic.at<double>(1, 3) += config.y;
    lidar_extrinsic.at<double>(2, 3) += config.z;

}

ToImage::ToImage(){
//    doInit();
}

void ToImage::doInit(const std::string& extrinsic_path) {

    cv::FileStorage fs(extrinsic_path, cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cout << "cannot open " << extrinsic_path << std::endl << std::endl;
    }

    fs["CameraExtrinsicMat"] >> camera_extrinsic_;
    fs["CameraMat"] >> camera_intrinsic_;
    fs["DistCoeff"] >> distcoeff_;

    invR_ = camera_extrinsic_(cv::Rect(0, 0, 3, 3)).t();
    invT_ = -invR_ * (camera_extrinsic_(cv::Rect(3, 0, 1, 3)));
}

void ToImage::adjust_extrinsic(ExtrinsicConfig config) {

    std::cout << "config: " << config.x << ", " << config.y << ", " << config.z << ","
                            << config.roll << ", " << config.pitch << ", " << config.yaw;

    std::cout << "lidar_extrinsic = \n" << camera_extrinsic_ << std::endl;
    cv::Mat euler = rot2euler(camera_extrinsic_);
    std::cout << "euler = " << euler;
    euler.at<double>(0) += config.pitch;
    euler.at<double>(1) += config.roll;
    euler.at<double>(2) += config.yaw;

    cv::Mat camera_extrinsic;
    camera_extrinsic_.copyTo(camera_extrinsic);

    euler2rot(euler, camera_extrinsic, config);

    std::cout << "new extrinsic = \n" << camera_extrinsic << std::endl << std::endl << std::endl;

    invR_ = camera_extrinsic(cv::Rect(0, 0, 3, 3)).t();
    invT_ = -invR_ * (camera_extrinsic(cv::Rect(3, 0, 1, 3)));
}


void ToImage::point2image(
        const PointType &point_in,
        Eigen::Vector3d &point_image) {

    PointCloud point_cloud;
    point_cloud.push_back(point_in);
    std::vector<Eigen::Vector3d> points_image;

    pointcloud2image(point_cloud, points_image);
    if(points_image.size() > 0){
        point_image = points_image.at(0);
    }else{
        std::cout << "????";
    }
}

void ToImage::pointcloud2image(
        const PointCloud &point_cloud,
        std::vector<Eigen::Vector3d> &points_image) {
    points_image.clear();

    PointCloud point_selected;
    for(auto pt : point_cloud){
        if(pt.x < 0){
            continue;
        }
        point_selected.push_back(pt);
    }

    cv::Mat camera_points(point_selected.size(), 3, CV_64F);
    for(int i = 0; i < point_selected.size(); ++i){

        camera_points.at<double>(i, 0) = double(point_selected.points[i].x);
        camera_points.at<double>(i, 1) = double(point_selected.points[i].y);
        camera_points.at<double>(i, 2) = double(point_selected.points[i].z);
    }

    camera_points = camera_points * invR_.t() + cv::Mat::ones(point_selected.size(), 1, CV_64F) * invT_.t();


//    cv::Mat t_n(point_selected.size(), 3, CV_64F);
//    for(int i = 0; i < point_selected.size(); ++i){
//        t_n.at<double>(i, 0) = t_.at<double>(0);
//        t_n.at<double>(i, 1) = t_.at<double>(1);
//        t_n.at<double>(i, 2) = t_.at<double>(2);
//    }
//
//    camera_points = ((R_ * camera_points.t()).t() + t_n);

    /// 不要畸变
    cv::Mat proj_corners = (camera_intrinsic_ * camera_points.t()).t();
    for(int i = 0; i < point_selected.size(); ++i){
        Eigen::Vector3d point_img;
        point_img[0] = proj_corners.at<double>(i, 0) / proj_corners.at<double>(i, 2);
        point_img[1] = proj_corners.at<double>(i, 1) / proj_corners.at<double>(i, 2);
        point_img[2] = point_selected[i].intensity;
        points_image.push_back(point_img);
    }


//    std::vector<cv::Point2d> points_projection;
//    points_projection.resize(point_selected.size());
//    for(int i = 0; i < point_selected.size(); ++i){
//        double tmp_x = camera_points.at<double>(i, 0) / camera_points.at<double>(i, 2);
//        double tmp_y = camera_points.at<double>(i, 1) / camera_points.at<double>(i, 2);
//        double r2 = tmp_x*tmp_x + tmp_y*tmp_y;
//        double tmp_dist =1 +
//                         distcoeff_.at<double>(0) * r2 +
//                         distcoeff_.at<double>(1) * r2 * r2 +
//                         distcoeff_.at<double>(4) * r2 * r2 * r2;
//        points_projection[i].x = tmp_x*tmp_dist + 2*distcoeff_.at<double>(2)*tmp_x*tmp_y + distcoeff_.at<double>(3)*(r2 + 2 * tmp_x * tmp_x);
//        points_projection[i].y = tmp_y*tmp_dist + distcoeff_.at<double>(2) * (r2 + 2 * tmp_y * tmp_y) + 2*distcoeff_.at<double>(3)*tmp_x*tmp_y;
//        Eigen::Vector3d point_img;
//        point_img[0] = camera_intrinsic_.at<double>(0, 0) * points_projection[i].x + camera_intrinsic_.at<double>(0, 2);
//        point_img[1] = camera_intrinsic_.at<double>(1, 1) * points_projection[i].y + camera_intrinsic_.at<double>(1, 2);
//        point_img[2] = point_selected.points[i].intensity;
//        points_image.push_back(point_img);
//    }

}

void ToImage::pointcloud2image(
        const PointCloud &point_cloud,
        std::vector<cv::Point2f> &points_image) {
    points_image.clear();

    PointCloud point_selected;
    for(auto pt : point_cloud){
        if(pt.x < 0){
            continue;
        }
        point_selected.push_back(pt);
    }

    cv::Mat camera_points(point_selected.size(), 3, CV_64F);
    for(int i = 0; i < point_selected.size(); ++i){

        camera_points.at<double>(i, 0) = double(point_selected.points[i].x);
        camera_points.at<double>(i, 1) = double(point_selected.points[i].y);
        camera_points.at<double>(i, 2) = double(point_selected.points[i].z);
    }

    camera_points = camera_points * invR_.t() + cv::Mat::ones(point_selected.size(), 1, CV_64F) * invT_.t();

    std::vector<cv::Point2d> points_projection;
    points_projection.resize(point_selected.size());
    for(int i = 0; i < point_selected.size(); ++i){
        double tmp_x = camera_points.at<double>(i, 0) / camera_points.at<double>(i, 2);
        double tmp_y = camera_points.at<double>(i, 1) / camera_points.at<double>(i, 2);
        double r2 = tmp_x*tmp_x + tmp_y*tmp_y;
        double tmp_dist =1 +
                         distcoeff_.at<double>(0) * r2 +
                         distcoeff_.at<double>(1) * r2 * r2 +
                         distcoeff_.at<double>(4) * r2 * r2 * r2;
        points_projection[i].x = tmp_x*tmp_dist + 2*distcoeff_.at<double>(2)*tmp_x*tmp_y + distcoeff_.at<double>(3)*(r2 + 2 * tmp_x * tmp_x);
        points_projection[i].y = tmp_y*tmp_dist + distcoeff_.at<double>(2) * (r2 + 2 * tmp_y * tmp_y) + 2*distcoeff_.at<double>(3)*tmp_x*tmp_y;
        cv::Point2f point_img;
        point_img.x = camera_intrinsic_.at<double>(0, 0) * points_projection[i].x + camera_intrinsic_.at<double>(0, 2);
        point_img.y = camera_intrinsic_.at<double>(1, 1) * points_projection[i].y + camera_intrinsic_.at<double>(1, 2);
        points_image.push_back(point_img);
    }
}

void ToImage::point2image(
        const PointType &point_in,
        cv::Point2f &point_image) {
    PointCloud point_cloud;
    point_cloud.push_back(point_in);
    std::vector<cv::Point2f> points_image;

    pointcloud2image(point_cloud, points_image);
    if(points_image.size() > 0){
        point_image = points_image.at(0);
    }else{
        std::cout << "????";
    }
}

