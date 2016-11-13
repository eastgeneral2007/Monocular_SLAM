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

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include <time.h>
#include <dirent.h>
#include "Frame.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "KeyFrame.h"
#include "MapPoint.h"

using namespace std;
using namespace cv;

class DataManager{
public:
    // camera intrinsics
    cv::Mat camera_intrinsics;

    // each original frame object read in and processed
    vector<Frame> frames;

    // each frames' reference KeyFrame
    vector<KeyFrame*> reference_keyframes;

    // each frames' Pose
    vector<cv::Mat> relative_frame_poses;

    // last frame and cur frame, for tracker to process
    Frame last_frame;
    Frame cur_frame;

    // indicate if last frame processed, so that loop_mapper and loop_closer will be fired
    bool new_frame_inserted;
    // last new key frame inserted
    KeyFrame last_new_key_frame;

    // The following is for Map
    vector<MapPoint*> all_map_points;
    vector<KeyFrame*> all_keyframes;
    vector<MapPoint*> reference_map_points;



};

#endif