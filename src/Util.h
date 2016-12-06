//
// Created by JimXing on 12/11/16.
// For optimisation, Bundle Adjustment + Pose Graph Estimation

#ifndef MONOCULAR_SLAM_UTIL_H
#define MONOCULAR_SLAM_UTIL_H
#include "Frame.h"
#include "DataManager.h"
#include <vector>

using namespace std;

class Util {
public:

    // util function to compute F from two general frames (actually just Frame here, since every frame now is frame)
    cv::Mat static ComputeF(Frame f1, Frame f2);

    // util function for optimisation
    void static BundleAdjustment(DataManager &data, vector<Frame> &frames, vector<MapPoint> &map_points,
                                 int n_iterations = 5, bool pb_stop_flag = false, const unsigned long n_loop_kf = 0,
                                 const bool b_robust = true);

    void static GlobalBundleAdjustemnt(DataManager & data, int n_iterations = 10, bool pb_stop_flag= false, const unsigned long n_loop_kf=0,
                                       const bool b_robust = true);

    void static LocalBundleAdjustment(Frame* frame, bool *pb_stop_flag, DataManager & data);

    void static PoseBundleAdjustment(Frame &frame, DataManager &data, int n_round = 5);

    // helper functions dealing with DataManager quering

    // find the frame corresponding to target id, null returned if not found
    static Frame* findFrameById(vector<Frame> &frames, int target_id);
    static MapPoint* findMapPointById(vector<MapPoint> &map_points, int target_id);

    /*
    load temporary Frames and MapPoints for testing 
    **/
    // Helper function to load frames from .csv files
    static vector<Frame> loadFrames(string frame_filename);
    // Helper function to load map points from .csv files
    static vector<MapPoint> loadMapPoints(string map_points_filename);
    // load only frame id + Optimised Rt from .csv files
    static vector<Frame> loadFramesOnlyIdRt(string frame_filename);
    // load only mappoint id + Optimised XYZ from .csv files
    static vector<MapPoint> loadMapPointsOnlyIdXYZ(string map_points_filename);
    // load a frame id to image name mapping for externel inputs
    static map<int, string> loadFrameIdToImageFileName(string info_filename);

};


#endif //MONOCULAR_SLAM_UTIL_H
