//
// Created by lgj on 2019/12/17.
//

#include "utils.h"


int angle_normalized(
        double& angle,
        const double& angle_min
)
{
    while (angle < angle_min)
    {
        angle += M_2PI;
    }

    double angle_max = angle_min + M_2PI;
    while (angle >= angle_max)
    {
        angle -= M_2PI;
    }

    return 0;
}

//// 从四元数翻成欧拉角
int fromQuatToEular(
        const geometry_msgs::Quaternion& rot,
        double *yaw,
        double *pitch,
        double *roll
)
{
    double yaw_v;
    double pitch_v;
    double roll_v;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(rot, quat); // from msg to quat
    tf::Matrix3x3(quat).getRPY(roll_v, pitch_v, yaw_v); // from quat to angles

    angle_normalized(roll_v);
    angle_normalized(pitch_v);
    angle_normalized(yaw_v);

    if (pitch != NULL)
    {
        *pitch = pitch_v;
    }
    if (roll != NULL)
    {
        *roll = roll_v;
    }
    if (yaw != NULL)
    {
        *yaw = yaw_v;
    }

    return 0;
}


cv::Mat fromHprToMat(
        const double& heading,
        const double& pitch,
        const double& roll,
        const double& delta_x,
        const double& delta_y,
        const double& delta_z
)
{
    cv::Mat mat = cv::Mat::zeros(3, 4, CV_64F);

    cv::Mat heading_m = (cv::Mat_<double>(3, 3) << cos(heading), -sin(heading), 0.0,
            sin(heading), cos(heading), 0.0,
            0.0, 0.0, 1.0);
    cv::Mat pitch_m= (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch));
    cv::Mat roll_m  = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll));

    mat(cv::Rect(0, 0, 3, 3)) = heading_m * pitch_m * roll_m;

    mat.at<double>(0, 3) = delta_x;
    mat.at<double>(1, 3) = delta_y;
    mat.at<double>(2, 3) = delta_z;

    return mat;
}
