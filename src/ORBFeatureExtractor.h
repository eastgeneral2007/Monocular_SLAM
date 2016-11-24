//
// ORBFeatureExtractor.h
//
// Extract ORB feature of each frame and feed 
// the extracted feature descriptor and position
// into the data manager.
//
// @Yu

#ifndef ORBFEATUREEXTRACTOR_H
#define ORBFEATUREEXTRACTOR_H

#include "CommonCV.h"
#include "ProcessingNode.h"

class ORBFeatureExtractor: public ProcessingNode
{
public:
	ORBFeatureExtractor():ProcessingNode("ORBFeatureExtractor") {}
	virtual void process(DataManager& data, int frameIdx) override;
	virtual bool validationCheck(DataManager& data, int frameIdx) override;
private:
	OrbFeatureDetector detector;
	OrbDescriptorExtractor extractor;
};

#endif