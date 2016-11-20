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
    static g2o::SE3Quat cvMatToSE3Quat(Mat mat);

};


#endif //SUPERBUILD_CONVERTER_H
