//
// Created by JimXing on 13/11/16.
//

#ifndef MONOCULAR_SLAM_LOOPCLOSER_H
#define MONOCULAR_SLAM_LOOPCLOSER_H
#include "ProcessingNode.h"
#include <list>
#include "DataManager.h"
#include <set>

class LoopCloser : public ProcessingNode{
public:
    typedef pair<set<KeyFrame*>,int> consistent_group;

    // local reference to data manager, so that no need to have "DataManager & data" registration in all methods
    DataManager & data_reference;

    // The following attributes may or may not be needed depending on our implementation
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    KeyFrame* current_kf;
    KeyFrame* matched_kf;

    std::vector<consistent_group> consistent_groups;
    std::vector<KeyFrame*> enough_consistent_candidates;
    std::vector<KeyFrame*> current_connected_kfs;
    std::vector<MapPoint*> current_matched_points;
    std::vector<MapPoint*> Loop_map_points;

public:
    // Constructor
    LoopCloser(const string &name, DataManager &data_reference);

    // loopCloser main function
    virtual void process(DataManager& data) override {}

    void RunGlobalBundleAdjustment(unsigned long n_loop_kf);

    void InsertKeyFrame(KeyFrame *keyframe);

    bool CheckNewKeyFrames();

    bool DetectLoop();

    void SearchAndFuse();

    void CorrectLoop();

};


#endif //MONOCULAR_SLAM_LOOPCLOSER_H
