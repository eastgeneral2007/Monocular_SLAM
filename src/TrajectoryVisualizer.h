//
// TrajectoryVisualizer.h
//
// Trajectory Visualization 
//
// @Sharon


#ifndef TRAJRECTORYVISUALIZER_H
#define TRAJRECTORYVISUALIZER_H

#include "Common.h"
#include "CommonCV.h"
#include "CommonBoost.h"
#include "ProcessingNode.h"

class TrajectoryVisualizer: public ProcessingNode
{
public:
	TrajectoryVisualizer():ProcessingNode("TrajectoryVisualizer") {}
	virtual void process(DataManager& data, int frameIdx) override;
	virtual bool validationCheck(DataManager& data, int frameIdx) override;
};

#endif