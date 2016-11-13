//
// Frame.h
// A central structure for storing per-frame data. 
//
// The design pattern is called CAS(Central Commmon Analysis) where 
// processing  modules in different stages of the work flow generate 
// and feed the analysis result into this sturct.
//
// @Yu

#ifndef FRAME_H
#define FRAME_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "MapPoint.h"

using namespace std;
using namespace cv;

struct Feature
{
    Point2d position;
    vector<unsigned char> descriptor;
};

struct FrameMeta
{
    double timestamp;
    int frameID;
    string framename;
};

typedef Mat RawBuffer;

struct Frame {
public:
    // Frame Meta Info
    FrameMeta meta;

    // Frame info, pixel info
    RawBuffer frame;

    // Relative pose, camera centre
    cv::Mat R_relative;
    cv::Mat T_relative;
    cv::Mat camera_centre;
    cv::Mat camera_pose; // R|T

    // features
    vector<Feature> features;

    // mapPoints associated to feature points, NULL if no association
    std::vector<MapPoint*> feature_to_mappoints;

    bool operator < (const Frame & frameB) const {
        return frameB.meta.timestamp > meta.timestamp;
    }
};

#endif