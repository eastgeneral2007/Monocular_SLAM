//
// FeatureExtractor.h
//
// Extract ORB feature of each frame and feed 
// the extracted feature descriptor and position
// into the data manager.
//
// @Yu

#ifndef FeatureExtractor_H
#define FeatureExtractor_H

#include "CommonCV.h"
#include "ProcessingNode.h"

class FeatureExtractor: public ProcessingNode
{
public:
	FeatureExtractor():ProcessingNode("FeatureExtractor") {}
	virtual void process(DataManager& data, int frameIdx) override;
	virtual bool validationCheck(DataManager& data, int frameIdx) override;
private:
	OrbFeatureDetector detector;
	OrbDescriptorExtractor extractor;
};

#endif