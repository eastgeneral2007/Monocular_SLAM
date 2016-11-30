//
// MapPoint.h
// 
// 3D point obtained from triangulation.
//
// @Jim, Yu

#ifndef MONOCULAR_SLAM_MAPPOINT_H
#define MONOCULAR_SLAM_MAPPOINT_H

#include "Common.h"
#include "CommonCV.h"

class KeyFrame;

class MapPoint {

public:
    long unsigned int id; // identifier
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
