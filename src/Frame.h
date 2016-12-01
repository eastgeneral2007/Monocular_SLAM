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

struct Features
{
    /* position of N features */
    vector<Point2d> positions;

    /* NxM mat storing the M-length descriptors of N features*/
    Mat descriptors;  // for ORB, each row is of size 32 (128 byte)
    
    /* the map point each feature corresponds to */
    vector<MapPoint*> mapPoints;
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
    RawBuffer frameBuffer;

    // Relative R|t matrix
    cv::Mat Rt;

    // features
    Features features;

    bool operator < (const Frame & frameB) const {
        return frameB.meta.timestamp > meta.timestamp;
    }
};

#endif