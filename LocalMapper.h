//
// Created by JimXing on 13/11/16.
//

#ifndef MONOCULAR_SLAM_LOCALMAPPER_H
#define MONOCULAR_SLAM_LOCALMAPPER_H

#include "ProcessingEngine.h"
#include <list>
#include "DataManager.h"

class LocalMapper : public ProcessingEngine{
public:
    std::list<KeyFrame*> new_keyframes;

    KeyFrame* current_keyframe;

    std::list<MapPoint*> recent_added_map_points;

    // local reference to data manager, so that no need to have "DataManager & data" registration in all methods
    DataManager & data_reference;

public:
    // Constructor
    LocalMapper(const string &name, KeyFrame *current_keyframe, DataManager &data_reference);

    // localMapper main function
    virtual void process(DataManager& data) override;

    bool CheckNewKeyFrames();

    void ProcessNewKeyFrame();

    void CreateNewMapPoints();

    // remove redundant map points
    void MapPointCulling();

    void SearchInNeighbors();

    // remove redundant key frames
    void KeyFrameCulling();


};


#endif //MONOCULAR_SLAM_LOCALMAPPER_H
