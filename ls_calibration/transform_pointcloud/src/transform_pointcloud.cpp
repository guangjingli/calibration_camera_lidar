//
// Created by lgj on 2019/12/17.
//

#include "utils.h"

bool is_calibrated_ = false;
tf2_ros::Buffer tfBuffer_;
cv::Mat lidar_extrinsic_;

std::string source_frame = "lidar_left";
ros::Publisher publisher_pc;


void tfStaticCallback(const geometry_msgs::TransformStamped& tf){
    double pitch, roll, yaw;
    fromQuatToEular(tf.transform.rotation,
                    &yaw,
                    &pitch,
                    &roll
    );
    lidar_extrinsic_ = fromHprToMat(yaw, pitch, roll,
                                   tf.transform.translation.x,
                                   tf.transform.translation.y,
                                   tf.transform.translation.z
    );
    is_calibrated_ = true;
    char text[1024];
    sprintf(text, "received the tf msg frame_id =%s,child_frame_id =%s:yaw=%1.2f, pitch=%1.2f, roll=%1.2f, x=%1.2f, y=%1.2f, z=%1.2f",
            std::string(tf.header.frame_id), std::string(tf.child_frame_id),
            yaw * degree_per_rad, pitch * degree_per_rad, roll * degree_per_rad,
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);
    ROS_INFO(text);
}

void callback(const sensor_msgs::PointCloud2ConstPtr pointcloud_msg){
    if(!is_calibrated_){
        tf2_ros::TransformListener tfStaticListener(tfBuffer_);
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer_.lookupTransform("smartcar", source_frame, ros::Time(0), ros::Duration(1.0));
            tfStaticCallback(transformStamped);
        }
        catch(tf2::TransformException &ex){
            ROS_ERROR("No received extrinsic params,please check");
            return;
        }
    }
    double a00 = lidar_extrinsic_.at<double>(0, 0);
    double a01 = lidar_extrinsic_.at<double>(0, 1);
    double a02 = lidar_extrinsic_.at<double>(0, 2);
    double a03 = lidar_extrinsic_.at<double>(0, 3);

    double a10 = lidar_extrinsic_.at<double>(1, 0);
    double a11 = lidar_extrinsic_.at<double>(1, 1);
    double a12 = lidar_extrinsic_.at<double>(1, 2);
    double a13 = lidar_extrinsic_.at<double>(1, 3);

    double a20 = lidar_extrinsic_.at<double>(2, 0);
    double a21 = lidar_extrinsic_.at<double>(2, 1);
    double a22 = lidar_extrinsic_.at<double>(2, 2);
    double a23 = lidar_extrinsic_.at<double>(2, 3);

    pcl::PointCloud<pcl::PointXYZI> pc_out;

    uint32_t x_offset = pointcloud_msg->fields[0].offset;
    uint32_t y_offset = pointcloud_msg->fields[1].offset;
    uint32_t z_offset = pointcloud_msg->fields[2].offset;
    uint32_t i_offset = pointcloud_msg->fields[3].offset;
    for(uint32_t point_start_byte = 0, counter = 0;
        point_start_byte < pointcloud_msg->data.size();
        point_start_byte += pointcloud_msg->point_step, ++counter){
        pcl::PointXYZI pt;
        pcl::PointXYZI pt_transed;

        pt.x = BytesTo<float>(pointcloud_msg->data, point_start_byte + x_offset);
        pt.y = BytesTo<float>(pointcloud_msg->data, point_start_byte + y_offset);
        pt.z = BytesTo<float>(pointcloud_msg->data, point_start_byte + z_offset);
        pt.intensity = BytesTo<float>(pointcloud_msg->data, point_start_byte + i_offset);

        pt_transed.x = a00 * pt.x + a01 * pt.y + a02 * pt.z + a03;
        pt_transed.y = a10 * pt.x + a11 * pt.y + a12 * pt.z + a13;
        pt_transed.z = a20 * pt.x + a21 * pt.y + a22 * pt.z + a23;
        pt_transed.intensity = pt.intensity;

////        if(pt_transed.z < 0.5)
//        {
//            continue;
//        }
        pc_out.points.push_back(pt_transed);
    }

    pc_out.header.frame_id = "smartcar";
    pc_out.header.stamp = pcl_conversions::toPCL(pointcloud_msg->header.stamp);
    publisher_pc.publish(pc_out);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "transform_point_cloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string velodyne_topic = pnh.param("velodyne_sub_topic", std::string("/vlp32_0/velodyne_points"));
    std::string velodyne_publish_topic = pnh.param("velodyne_pub_topic", std::string("/vlp32_0/pc_transformed"));
    source_frame = pnh.param("source_frame", std::string("lidar_left"));

    publisher_pc = pnh.advertise<sensor_msgs::PointCloud2>(velodyne_publish_topic, 2);
    ros::Subscriber pc_sub_ = pnh.subscribe(velodyne_topic, 2, callback);

    ros::spin();
    return 0;
}








