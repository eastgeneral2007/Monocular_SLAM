//
// Created by JimXing on 19/11/16.
//

#ifndef SUPERBUILD_CONVERTER_H
#define SUPERBUILD_CONVERTER_H

#include "Common.h"
#include "CommonCV.h"
#include "Commong2o.h"

#include<Eigen/Dense>

using namespace std;
using namespace cv;

class Converter {
public:
    // conversion between cv::Mat and g2o::SE3Quat
    static g2o::SE3Quat cvMatToSE3Quat(const Mat mat);
    static Mat SE3QuatToCvMat(const g2o::SE3Quat &SE3);

    // conversion between cv::Mat and Eigen::Matrix<double,3,1>
    static Mat vector3DToCvMat(const Eigen::Matrix<double,3,1> &m);
    static Eigen::Matrix<double,3,1> cvMatToVector3d(const Mat mat);

    // conversion between Eigen and cv Mat
    static Mat eigenMatrixToCvMat(const Eigen::Matrix<double,4,4> &m);
    static Eigen::Matrix<double,4,4> cvMatToEigenMatrix(const Mat mat);

};


#endif //SUPERBUILD_CONVERTER_H
