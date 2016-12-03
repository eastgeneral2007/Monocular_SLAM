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
    // local reference to data manager, so that no need to have "DataManager & data" registration in all methods
    DataManager & data_reference;

    typedef pair<set<Frame*>,int> consistent_group;

    // The following attributes may or may not be needed depending on our implementation
    std::list<Frame*> mlpLoopFrameQueue;

    Frame* current_kf;
    Frame* matched_kf;

    std::vector<consistent_group> consistent_groups;
    std::vector<Frame*> enough_consistent_candidates;
    std::vector<Frame*> current_connected_kfs;
    std::vector<MapPoint*> current_matched_points;
    std::vector<MapPoint*> Loop_map_points;

public:
    // Constructor
    LoopCloser(const string &name, DataManager &data_reference);

    // loopCloser main function
    virtual void process(DataManager& data) override {}

    void RunGlobalBundleAdjustment(unsigned long n_loop_kf);

    void Run();

    void InsertFrame(Frame *keyframe);

    bool CheckNewFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse();

    void CorrectLoop();

    void NBestMatches(Mat descriptors1, Mat descriptors2, unsigned int n, vector<vector<float> > & distances, vector<vector<int> > & indices);

    void TestUnit();

};


#endif //MONOCULAR_SLAM_LOOPCLOSER_H
