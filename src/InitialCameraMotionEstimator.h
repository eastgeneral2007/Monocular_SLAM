//
// InitialCameraMotionEstimator.h
//
// This stage performs camera motion estimation
// for each frame based on the feature matching 
// with the previous frame. An initial camera
// matrix, which transforms from word space to
// camera space, is estimated. 
//
// @Yu

#ifndef INITIALCAMERAMOTIONESTIMATOR_H
#define INITIALCAMERAMOTIONESTIMATOR_H

#include "ProcessingNode.h"
#include "CommonCV.h"

// #define DEBUG_INITIALCAMERAMOTIONESTIMATOR

class InitialCameraMotionEstimator: public ProcessingNode
{
public:
	InitialCameraMotionEstimator():ProcessingNode("InitialCameraMotionEstimator"), matcher(NORM_HAMMING, true) {}
	virtual void process(DataManager& data, int frameIdx);
	virtual bool validationCheck(DataManager& data, int frameIdx);
private:
	BFMatcher matcher;
};

#endif