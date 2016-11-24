//
// Created by JimXing on 13/11/16.
//

#ifndef MONOCULAR_SLAM_TRACKER_H
#define MONOCULAR_SLAM_TRACKER_H


#include "ProcessingNode.h"

class Tracker : public ProcessingNode{
public:
    // local reference to data manager, so that no need to have "DataManager & data" registration in all methods
    DataManager & data_reference;
public:
    // Constructor
    Tracker(DataManager &data_reference);

    // invoke detector on two frames, estimated pose
    virtual void process(DataManager& data) override {}

    // estimate pose, given two frame, estimate R, T
    cv::Mat poseEstimation(Frame f1, Frame f2);

    // track local map
    void trackLocalMap();

    // detect if need to insert key frame
    bool needInsertNewKeyFrame();

    // insert new frame
    void insertNewKeyFrame();

    // create initial map from the first two frames:
    // 1. create mapPoints, associate with KeyFrames
    // 2. Add to map in DataManager
    // 3. Update KeyFrame covisibility graph in DataManager
    // 4. run Global BA
    void createInitialMap();

    // TODO: check relationship between localMapping Class and Map Class


};


#endif //MONOCULAR_SLAM_TRACKER_H
