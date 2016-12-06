//
// MapPoint.h
// 
// 3D point obtained from triangulation.
//
// @Jim, Yu

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "Common.h"
#include "CommonCV.h"
// #include "DataManager.h"

class Frame;

class MapPoint {
friend class Util;
public:
    MapPoint(Point3d worldPosition, int id): worldPosition(worldPosition), id(id) {}
    long unsigned int id; // identifier
    Point3d worldPosition; // 3D world coordinate
public:
    void addObservingFrame(int frameIdx, int featureIdx);
    int getFeatureIdxFromObservingFrame(int frameIdx);
    void deleteObservingFrame(int frameIdx);
    map<int, int> observerToIndex; // map of observing Frame to the index of feature
};


#endif //MONOCULAR_SLAM_MAPPOINT_H
