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

class Frame;

class MapPoint {

public:
    long unsigned int id; // identifier
    Point3d worldPosition; // 3D world coordinate
    std::map<Frame*, int> observerToIndex; // map of observing Frame to the index of feature
public:
    void addObservingFrame(Frame* frame, int featureIdx);
    int  getFeatureIdxFromObservingFrame(Frame* frame);
    void deleteObservingFrame(Frame* frame);
};


#endif //MONOCULAR_SLAM_MAPPOINT_H
