//
//
// Monocular SLAM
// Following the pipeline of ORB_SLAM with optimisation framework implemented in OPT
//
// Xiaohan Jin, Yifan Xing, Yu Mao @ CMU
// 11/01/2016
//

#ifndef MONOCULAR_SLAM_DATA_MANAGER_H
#define MONOCULAR_SLAM_DATA_MANAGER_H

#include "Common.h"
#include "Frame.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "MapPoint.h"

using namespace std;
using namespace cv;

class DataManager{
public:
    // camera intrinsics (deprecated)
    cv::Mat camera_intrinsics; // should not be used in the code anymore

    // each original frame object read in and processed
    vector<Frame> frames;

    // each frames' Pose
    vector<cv::Mat> relative_frame_poses;

    // 3D map points
    vector<MapPoint> mapPoints;
};

#endif