//
// Created by lgj on 2019/11/18.
//

#include "to_image.h"
#include "singleton.hpp"


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

