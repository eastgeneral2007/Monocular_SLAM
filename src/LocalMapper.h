//
// Created by JimXing on 13/11/16.
//

#ifndef MONOCULAR_SLAM_LOCALMAPPER_H
#define MONOCULAR_SLAM_LOCALMAPPER_H

#include "ProcessingNode.h"
#include <list>
#include "DataManager.h"

class LocalMapper : public ProcessingNode{
public:
    std::list<Frame*> new_frames;

    Frame* current_frame;

    std::list<MapPoint*> recent_added_map_points;

    // local reference to data manager, so that no need to have "DataManager & data" registration in all methods
    DataManager & data_reference;

public:
    // Constructor
    LocalMapper(const string &name, DataManager &data_reference);

    // localMapper main function
    virtual void process(DataManager& data) {}

    bool CheckNewFrames();

    void ProcessNewFrame();

    void CreateNewMapPoints();

    // remove redundant map points
    void MapPointCulling();

    void SearchInNeighbors();

    // remove redundant key frames
    void FrameCulling();


};


#endif //MONOCULAR_SLAM_LOCALMAPPER_H
