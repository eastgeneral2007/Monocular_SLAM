//
// Created by JimXing on 12/11/16.
// 3D MapPoints

#ifndef MONOCULAR_SLAM_MAPPOINT_H
#define MONOCULAR_SLAM_MAPPOINT_H
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class KeyFrame;

class MapPoint {

public:
    cv::Mat world_pos;// 3D world coordinate
    std::map<KeyFrame*, size_t> observer_to_index; // map of observing KeyFrame
    KeyFrame* reference_kf; // reference KF, the nearest one
    cv::Mat mean_view_dir; // mean viewing direction

public:

    void addObservingKeyFrame(KeyFrame* key_frame, size_t index);
    size_t getObservingKeyFrame(KeyFrame* key_frame);
    void deleteObservingKeyFrame(KeyFrame*);

};


#endif //MONOCULAR_SLAM_MAPPOINT_H
