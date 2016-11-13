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

    // Frame info
    RawBuffer frame;

    // features
    vector<Feature> features;

    bool operator < (const Frame & frameB) const {
        return frameB.meta.timestamp > meta.timestamp;
    }
};

#endif