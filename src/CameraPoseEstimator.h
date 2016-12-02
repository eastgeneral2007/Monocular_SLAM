//
// CameraPoseEstimator.h
//
// This stage performs camera motion estimation
// for each frame based on the feature matching 
// with the previous frame. An initial camera
// matrix, which transforms from word space to
// camera space, is estimated. 
//
// @Yu

#ifndef CAMERAPOSEESTIMATOR_H
#define CAMERAPOSEESTIMATOR_H

#include "ProcessingNode.h"
#include "CommonCV.h"

class CameraPoseEstimator: public ProcessingNode
{
public:
	CameraPoseEstimator():ProcessingNode("CameraPoseEstimator"){}
	virtual void process(DataManager& data, int frameIdx);
	virtual bool validationCheck(DataManager& data, int frameIdx);
private:
	void initialPoseEstimation(DataManager& data, int frameIdx);
	void pnpPoseEstimation(DataManager& data, int frameIdx);
};

#endif