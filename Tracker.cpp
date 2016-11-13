//
// Created by JimXing on 13/11/16.
//

#include "Tracker.h"
#include "Util.h"
cv::Mat Tracker::poseEstimation (Frame f1, Frame f2) {
    return Util::ComputeF(f1, f2);
}

Tracker::Tracker(const string &name, DataManager &data_reference) : ProcessingEngine(name),
                                                                    data_reference(data_reference) {}
