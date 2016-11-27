//
// Created by JimXing on 19/11/16.
//

#include <iostream>
#include "UnitTestg2o.h"
#include "../src/CommonCV.h"
#include "../src/Converter.h"

using namespace std;
using namespace cv;

void UnitTestg2o::unitTestCvMatToG2oSE3() {
    cout<< "Test unitTestCvMatToG2oSE3"<< endl;
    Mat R = (Mat_<double>(3,3) <<     0.9506,  0.2940,  -0.0998,
    -0.2845,    0.9535,   0.0993,
    0.1244,   -0.0660,    0.9900);
    Mat T = (Mat_<double>(3,1) << 2, 3, 1);
    cout << "R = " << endl << " " << R << endl << endl;
    cout << "T = " << endl << " " << T << endl << endl;

    Mat pose = Mat::zeros(3,4, CV_64F);
    R.copyTo(pose.colRange(0,3).rowRange(0,3)); // exclusive range second parameter
//    pose.at<double>(3,3) = 1.0;
    T.copyTo(pose.colRange(3,4).rowRange(0,3));
    cout << "pose = " << endl << " " << pose << endl << endl;

    g2o::SE3Quat pose_se3 = Converter::cvMatToSE3Quat(pose);
    cout << "pose_se3 = "<< endl << pose_se3 << endl;
}
