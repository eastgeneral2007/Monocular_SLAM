//
// Created by JimXing on 12/11/16.
//

#ifndef MONOCULAR_SLAM_KEYFRAME_H
#define MONOCULAR_SLAM_KEYFRAME_H

#include<stdio.h>
#include<cstdlib>
#include <string.h>
#include <vector>
#include "Frame.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace std;
using namespace cv;

class MapPoint;

class KeyFrame: public Frame {

public:
    // ID
    long unsigned int id;

    // neighbouring KeyFrames
    vector<MapPoint*> map_points;
    vector<KeyFrame*> connected_keyframes;
    map<KeyFrame*, int> conneted_keyframes_weights;

    //Essential Graph Related (MST + high covisibility edges + loop closure edges)
    bool isfirst;
    KeyFrame* mst_parent; // the connected previous node in MST
    vector<KeyFrame*> mst_children; // the children in MST
    vector<KeyFrame*> mst_loop_edges; // loop closure edges
    vector<KeyFrame*> mst_high_cov_edges; // high covisibility edges, TODO: check if need to be combined with this.mst_children

    // TODO: Do we need a pointer to DataManager here (or in the Frame class)??

};


#endif //MONOCULAR_SLAM_KEYFRAME_H
