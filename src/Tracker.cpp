//
// Created by JimXing on 13/11/16.
//

#include "Tracker.h"
#include "Util.h"
cv::Mat Tracker::poseEstimation (Frame f1, Frame f2) {
    return Util::ComputeF(f1, f2);
}

Tracker::Tracker(DataManager &data_reference) : ProcessingNode("Tracker"),
                                                                    data_reference(data_reference) {}
