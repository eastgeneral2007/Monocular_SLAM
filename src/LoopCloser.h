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

    int LastLoopKFid;

public:
    // Constructor
    LoopCloser(const string &name, DataManager &data_reference);

    // loopCloser main function
    virtual void process(DataManager& data) override {}

    void RunGlobalBundleAdjustment(unsigned long n_loop_kf);

    void Run();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse();

    void CorrectLoop();

    void NBestMatches(Mat descriptors1, Mat descriptors2, unsigned int n, vector<vector<float>>& distances, vector<vector<int>>& indices);

    void TestUnit();

};


#endif //MONOCULAR_SLAM_LOOPCLOSER_H
