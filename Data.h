#ifndef DATA_H
#define DATA_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

struct ORBFeature
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

    // ORBFeature features
    vector<int> ORBFeature;

    bool operator < (const Frame & frameB) const
    {
        return frameB.timestamp > timestamp;
    }
};

#endif