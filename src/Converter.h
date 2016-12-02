//
// Converter.h
//
// A utility class for converting primitive types across eigen, OpenCV and g2o.
//
// @Jim, Yu

#ifndef CONVERTER_H
#define CONVERTER_H

#include "Common.h"
#include "CommonCV.h"
#include "Commong2o.h"

#include<Eigen/Dense>

namespace Converter {
    // conversion between cv::Mat and g2o::SE3Quat
    g2o::SE3Quat cvMatToSE3Quat(const Mat mat);
    Mat SE3QuatToCvMat(const g2o::SE3Quat &SE3);

    // conversion between cv::Mat and Eigen::Matrix<double,3,1>
    Mat vector3DToCvMat(const Eigen::Matrix<double,3,1> &m);
    Eigen::Matrix<double,3,1> cvMatToVector3d(const Mat mat);

    // conversion between Point3d and Eigen::Matrix<double,3,1>
    Eigen::Matrix<double,3,1> point3dToVector3d(const Point3d p);
    Point3d vector3dToPoint3d(const Eigen::Matrix<double,3,1> p);

    // conversion between Eigen and cv Mat
    Mat eigenMatrixToCvMat(const Eigen::Matrix<double,4,4> &m);
    Eigen::Matrix<double,4,4> cvMatToEigenMatrix(const Mat mat);
};


#endif //SUPERBUILD_CONVERTER_H
