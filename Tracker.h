//
// Created by JimXing on 13/11/16.
//

#ifndef MONOCULAR_SLAM_TRACKER_H
#define MONOCULAR_SLAM_TRACKER_H


#include "ProcessingEngine.h"

class Tracker : public ProcessingEngine{
public:
    Tracker();

    // invoke detecter on two frames, estimated pose
    virtual void process(DataManager& data) override;

    // estimate pose, given two frame, estimate R, T
    void poseEstimation(Frame f1, Frame f2);

    // track local map
    void trackLocalMap(DataManager & data);

    // detect if need to insert key frame
    bool needDetectNewKeyFrame(DataManager & data);

    // insert new frame
    void insertNewKeyFrame(DataManager & data);

    // create initial map from the first two frames:
    // 1. create mapPoints, associate with KeyFrames
    // 2. Add to map in DataManager
    // 3. Update KeyFrame covisibility graph in DataManager
    // 4. run Global BA
    void createInitialMap(DataManager & data);

    // TODO: check relationship between localMapping Class and Map Class


};


#endif //MONOCULAR_SLAM_TRACKER_H
