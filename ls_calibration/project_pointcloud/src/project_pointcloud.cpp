//
// Created by lgj on 2019/12/18.
//

#include "to_image.h"
#include "cyc_array.h"
#include "singleton.hpp"
#include "utils.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <project_pointcloud/adjust_Config.h>

std::string left_source_frame = "lidar_left";
std::string right_source_frame = "lidar_right";

bool isImage = false;
CycArray<sensor_msgs::ImageConstPtr> image_msg_;
ros::Publisher publisher_left;
ros::Publisher publisher_right;

bool left_is_calibrated_ = false;
bool right_is_calibrated_ = false;
cv::Mat lidar_left_extrinsic_;
cv::Mat lidar_right_extrinsic_;
tf2_ros::Buffer tfBuffer_;

ExtrinsicConfig config_;
ToImage to_image_;

void tfStaticCallback(const geometry_msgs::TransformStamped& tf,
        cv::Mat& lidar_extrinsic, bool& is_calibrated_,
        std::string& source_frame){
    double pitch, roll, yaw;
    fromQuatToEular(tf.transform.rotation,
                    &yaw,
                    &pitch,
                    &roll
    );
    lidar_extrinsic = fromHprToMat(yaw, pitch, roll,
                                         tf.transform.translation.x,
                                         tf.transform.translation.y,
                                         tf.transform.translation.z
    );
    is_calibrated_ = true;
    char text[1024];
    sprintf(text, "received the source_frame %s:yaw=%1.2f, pitch=%1.2f, roll=%1.2f, x=%1.2f, y=%1.2f, z=%1.2f",
            source_frame.c_str(),
            yaw * degree_per_rad, pitch * degree_per_rad, roll * degree_per_rad,
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);
    ROS_INFO(text);
}

int projectPoints2Image(const sensor_msgs::PointCloud2ConstPtr pointMsg,
                        const cv::Mat& lidar_extrinsic_,
                        const std::string source_frame,
                        ros::Publisher& publisher){
    PointCloud points;
    pcl::fromROSMsg(*pointMsg, points);

    sensor_msgs::ImageConstPtr image_temp;
    image_msg_.try_pop_lastest(image_temp);
    cv::Mat image;
    cv_bridge::CvImagePtr img_ptr;
    img_ptr = cv_bridge::toCvCopy(image_temp, sensor_msgs::image_encodings::BGR8);
    img_ptr->image.copyTo(image);

    //// 点云旋转
    for(size_t i = 0; i < points.points.size(); ++i){
        auto pt = points.points.at(i);
        points.points.at(i).x = lidar_extrinsic_.at<double>(0, 0) * pt.x +
                                lidar_extrinsic_.at<double>(0, 1) * pt.y +
                                lidar_extrinsic_.at<double>(0, 2) * pt.z +
                                lidar_extrinsic_.at<double>(0, 3);
        points.points.at(i).y = lidar_extrinsic_.at<double>(1, 0) * pt.x +
                                lidar_extrinsic_.at<double>(1, 1) * pt.y +
                                lidar_extrinsic_.at<double>(1, 2) * pt.z +
                                lidar_extrinsic_.at<double>(1, 3);
        points.points.at(i).z = lidar_extrinsic_.at<double>(2, 0) * pt.x +
                                lidar_extrinsic_.at<double>(2, 1) * pt.y +
                                lidar_extrinsic_.at<double>(2, 2) * pt.z +
                                lidar_extrinsic_.at<double>(2, 3);
    }

    double time_lidar = pointMsg->header.stamp.toSec();
    double time_image = image_temp->header.stamp.toSec();
    double time_diff = time_lidar - time_image;
    std::cout << source_frame << "_time - image_time = " << time_diff << std::endl;

    std::vector<Eigen::Vector3d> points_image;
    to_image_.adjust_extrinsic(config_);
    to_image_.pointcloud2image(points, points_image);

    for(size_t i = 0; i < points_image.size(); ++i){
        cv::Point pt;
        pt.x = points_image.at(i)[0];
        pt.y = points_image.at(i)[1];
        if(pt.x > 0 && pt.x < 1280 && pt.y > 0 && pt.y < 1080)
        {
            cv::circle(image, pt, 1, cv::Scalar(255, 0, 255), -1);
        }
    }

    sensor_msgs::ImagePtr msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    publisher.publish(msg);
}

void velodyneLeftCallback(const sensor_msgs::PointCloud2ConstPtr &pointsMsg){
    if(isImage){
        projectPoints2Image(pointsMsg, lidar_left_extrinsic_, left_source_frame ,publisher_left);
    }
}

void velodyneRightCallback(const sensor_msgs::PointCloud2ConstPtr &pointsMsg){
    if(isImage){
        projectPoints2Image(pointsMsg, lidar_right_extrinsic_, right_source_frame, publisher_right);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &imageMsg){
    image_msg_.update(imageMsg);
    isImage = true;
}

void dynamicConfigCallback(project_pointcloud::adjust_Config &config, uint32_t level){
    config_.x = config.x_adjust;
    config_.y = config.y_adjust;
    config_.z = config.z_adjust;
    config_.roll = config.roll_adjust;
    config_.pitch = config.pitch_adjust;
    config_.yaw = config.yaw_adjust;

}


int main(int argc, char** argv){
    ros::init(argc, argv, "project_pointcloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string extrinsic_path = pnh.param("extrinsic_path", std::string("~/extrinsic.yaml"));
    std::cout << extrinsic_path << std::endl << std::endl;

    std::string topic_left_sub = pnh.param("topic_left_sub", std::string("/vlp32_0/velodyne_points"));
    std::string topic_right_sub = pnh.param("topic_right_sub", std::string("/vlp32_2/velodyne_points"));
    std::string topic_image_sub = pnh.param("topic_image_sub", std::string("/darknet_ros/detection_visualization"));
    std::cout << topic_left_sub << "\t" << topic_right_sub << "\t" << topic_image_sub << std::endl;

    left_source_frame = pnh.param("left_source_frame", std::string("lidar_left"));
    right_source_frame = pnh.param("right_source_frame", std::string("lidar_right"));


    to_image_.doInit(extrinsic_path);

    publisher_left = nh.advertise<sensor_msgs::Image>("vlp32_0/image", 5);
    publisher_right = nh.advertise<sensor_msgs::Image>("vlp32_2/image", 5);

    dynamic_reconfigure::Server<project_pointcloud::adjust_Config> server;
    dynamic_reconfigure::Server<project_pointcloud::adjust_Config>::CallbackType f;
    f = boost::bind(&dynamicConfigCallback, _1, _2);
    server.setCallback(f);


    while(!(left_is_calibrated_ && right_is_calibrated_)){
        tf2_ros::TransformListener tfStaticListener(tfBuffer_);
        geometry_msgs::TransformStamped transformStamped;
        if(!left_is_calibrated_){
            try{
                transformStamped = tfBuffer_.lookupTransform("smartcar", left_source_frame, ros::Time(0), ros::Duration(1.0));
                tfStaticCallback(transformStamped, lidar_left_extrinsic_, left_is_calibrated_, left_source_frame);
            }
            catch(tf2::TransformException &ex){
                ROS_ERROR("No received extrinsic params,please check");
            }
        }
        if(!right_is_calibrated_){
            try{
                transformStamped = tfBuffer_.lookupTransform("smartcar", right_source_frame, ros::Time(0), ros::Duration(1.0));
                tfStaticCallback(transformStamped, lidar_right_extrinsic_, right_is_calibrated_, right_source_frame);
            }
            catch(tf2::TransformException &ex){
                ROS_ERROR("No received extrinsic params,please check");
            }
        }
    }



    ros::Subscriber subImage = nh.subscribe<sensor_msgs::Image>(topic_image_sub, 5, imageCallback);
    ros::Subscriber subPointsLeft = nh.subscribe<sensor_msgs::PointCloud2>(topic_left_sub, 5, velodyneLeftCallback);
    ros::Subscriber subPointsRight = nh.subscribe<sensor_msgs::PointCloud2>(topic_right_sub, 5, velodyneRightCallback);

    ros::spin();
    return 1;
}
