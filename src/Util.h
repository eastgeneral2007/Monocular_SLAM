//
// Created by JimXing on 12/11/16.
// For optimisation, Bundle Adjustment + Pose Graph Estimation

#ifndef MONOCULAR_SLAM_UTIL_H
#define MONOCULAR_SLAM_UTIL_H
#include "KeyFrame.h"
#include "DataManager.h"
#include <vector>

using namespace std;

class Util {
public:
    // util function to compute Fundamental matrix from two keyframes, actually just manipulation with R and T stored in the Keyframes
    cv::Mat static ComputeF12(KeyFrame* &f1, KeyFrame* &f2);

    // util function to compute F from two general frames (not KeyFrame)
    cv::Mat static ComputeF(Frame f1, Frame f2);

    // util function for optimisation
    void static BundleAdjustment(const std::vector<KeyFrame*> &keyframes, const std::vector<MapPoint*> &map_points,
                                 int n_iterations = 5, bool *pb_stop_flag= NULL, const unsigned long n_loop_kf=0,
                                 const bool b_robust = true);

    void static GlobalBundleAdjustemnt(DataManager & data, int n_iterations = 5, bool *pb_stop_flag= NULL, const unsigned long n_loop_kf=0,
                                       const bool b_robust = true);

    void static LocalBundleAdjustment(KeyFrame* keyframe, bool *pb_stop_flag, DataManager & data);

    // load temporary KeyFrames and MapPoints for testing
    vector<KeyFrame> static loadKeyFrames(string key_frame_filename);
    vector<MapPoint> static loadMapPointsAndAssociateKeyFrames(string map_points_filename, vector<KeyFrame>);




};


#endif //MONOCULAR_SLAM_UTIL_H
