//
// Created by JimXing on 19/11/16.
//
#include "Converter.h"

g2o::SE3Quat Converter::cvMatToSE3Quat (Mat mat) {
    Eigen::Matrix<double,3,3> R;
    R << mat.at<double>(0,0), mat.at<double>(0,1), mat.at<double>(0,2),
            mat.at<double>(1,0), mat.at<double>(1,1), mat.at<double>(1,2),
            mat.at<double>(2,0), mat.at<double>(2,1), mat.at<double>(2,2);

    Eigen::Matrix<double,3,1> t(mat.at<double>(0,3), mat.at<double>(1,3), mat.at<double>(2,3));

    return g2o::SE3Quat(R,t);
}