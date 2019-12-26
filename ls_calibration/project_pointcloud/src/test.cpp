//
// Created by lgj on 2019/12/18.
//

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>


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

cv::Mat euler2rot(cv::Mat euler){
    cv::Mat mat = cv::Mat::zeros(3, 3, CV_64F);

    Eigen::Matrix3d rotation;

    auto r_x = Eigen::AngleAxisd(euler.at<double>(0), Eigen::Vector3d::UnitZ());
//    std::cout << "r_x = " << r_x << std::endl;

    rotation =
            Eigen::AngleAxisd(euler.at<double>(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler.at<double>(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler.at<double>(0), Eigen::Vector3d::UnitX());

    mat.at<double>(0, 0) = rotation(0, 0);
    mat.at<double>(0, 1) = rotation(0, 1);
    mat.at<double>(0, 2) = rotation(0, 2);
    mat.at<double>(1, 0) = rotation(1, 0);
    mat.at<double>(1, 1) = rotation(1, 1);
    mat.at<double>(1, 2) = rotation(1, 2);
    mat.at<double>(2, 0) = rotation(2, 0);
    mat.at<double>(2, 1) = rotation(2, 1);
    mat.at<double>(2, 2) = rotation(2, 2);

    return mat;
}

int main(){

    std::string extrinsic_path = "/home/lgj/workspace/calibration_ws/extrinsic.yaml";

    cv::FileStorage fs(extrinsic_path, cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cout << "cannot open " << extrinsic_path << std::endl << std::endl;
    }

    cv::Mat camera_extrinsic_;
    cv::Mat camera_intrinsic_;
    cv::Mat distcoeff_;

    fs["CameraExtrinsicMat"] >> camera_extrinsic_;
    fs["CameraMat"] >> camera_intrinsic_;
    fs["DistCoeff"] >> distcoeff_;

    std::cout << "camera_extrinsic:\n" << camera_extrinsic_ << std::endl << std::endl;
    std::cout << "camera_intrinsic_:\n" << camera_intrinsic_ << std::endl << std::endl;
    std::cout << "distcoeff_:\n" << distcoeff_ << std::endl << std::endl;

    cv::Mat euler =  rot2euler(camera_extrinsic_);
    std::cout << "euler\n" << euler << std::endl << std::endl;

    cv::Mat rot = euler2rot(euler);
    std::cout << "rot\n" << rot << std::endl << std::endl;

}


