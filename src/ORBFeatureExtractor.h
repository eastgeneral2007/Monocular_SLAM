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
#include "ProcessingEngine.h"

class ORBFeatureExtractor: public ProcessingEngine
{
public:
	ORBFeatureExtractor():ProcessingEngine("ORBFeatureExtractor") {}
	virtual void process(DataManager& data, Frame& frame) override;
	virtual bool validationCheck(DataManager& data, Frame& frame) override;
private:
	OrbFeatureDetector detector;
	OrbDescriptorExtractor extractor;
};

#endif